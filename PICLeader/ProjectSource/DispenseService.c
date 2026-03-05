/****************************************************************************
 Module
   DispenseService.c

 Summary
   Dispense sequence using push-down arm and rotating bucket bottom.
   Bucket arm is OC4 and bucket bottom is OC3.
   Lower arm is OC5.

   Sequence:
     - On ES_DISPENSE_START:
         1) Lower arm down (RB6 / OC5) while bucket arm stays initialized
         2) Delay for arm clearance
         3) Move bucket arm into dispense mode (RA4 / OC4)
         4) Small settle delay
         5) Rotate bucket bottom (dump 90 first time, 180 second time)
         6) Wait for balls
         7) Raise lower arm
         8) Return bucket arm to initialized position

 Notes (my notes):
   - Positional servos (OC5/OC4) were snapping too fast because I was stepping
     directly to the target pulse/ticks in one write.
   - Fix: I slew the commands (small increments every ~20ms) so motion looks smooth.
   - I kept the same state machine logic/timing; the move durations come from the
     same timers I already had (T_PUSH_ARM_DOWN_MS, etc.).
****************************************************************************/

#include "DispenseService.h"
#include "SPILeaderService.h"
#include "CollectService.h"

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*============================== TIMING ==============================*/
#define T_PUSH_ARM_DOWN_MS          950u
#define T_AFTER_ARM_DOWN_DELAY_MS   250u   /* clearance delay before bucket arm moves */
#define T_BUCKET_ARM_MOVE_MS        900u   /* time for bucket arm to reach dispense pose */
#define T_BUCKET_ARM_SETTLE_MS      150u   /* small settle delay after bucket arm move */

#define T_ROTATE_BUCKET_90_MS       200u   /* tuned 90 degree rotation */
#define T_ROTATE_BUCKET_180_MS      400u
#define T_DISPENSE_WAIT_MS          1200u
#define T_PUSH_ARM_UP_MS            1500u
#define T_BUCKET_ARM_RETURN_MS      900u   /* return bucket arm to initialized pose */

/*=========================== SERVO POSITIONS ============================*/
/* OC5 : lower arm (RB6) */
#define US_PUSH_ARM_UP              2500u
#define US_PUSH_ARM_DOWN            600u   /* tuned */

/* OC3 : continuous rotation bucket bottom (RB10) */
#define US_BUCKET_STOP              1500u  /* tuned */
#define US_BUCKET_ROTATE_CW         1700u  /* speed command; reduce closer to STOP if too fast */

/* OC4 : bucket arm (RA4) -- tick values copied from CollectService */
#define BUCKET_ARM_INIT_TICKS       5500u
#define BUCKET_ARM_DISPENSE_TICKS   3000u

/*============================== TIMERS ==============================*/
#ifndef DISPENSE_TIMER
#define DISPENSE_TIMER              2u
#endif

/* Slew update timer (if this conflicts with your project, change the number) */
#ifndef SERVO_TIMER
#define SERVO_TIMER                 13u
#endif

/* update servos at ~50Hz so the motion is smooth */
#define SERVO_UPDATE_MS             20u

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE = 0,

  DISP_PUSH_ARM_DOWN,
  DISP_ARM_CLEAR_DELAY,

  DISP_BUCKET_ARM_TO_DISPENSE,
  DISP_BUCKET_ARM_SETTLE,

  DISP_ROTATE_BUCKET,
  DISP_WAIT_DROP,
  DISP_PUSH_ARM_UP,

  DISP_BUCKET_ARM_RETURN,
  DISP_DONE
} DispState_t;

static DispState_t CurState;
static uint8_t MyPriority;
static bool FirstDispense = true;

/*====================== PWM HELPERS =====================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);

static void Servo_OC3(uint16_t us);        /* bucket bottom */
static void Servo_OC5(uint16_t us);        /* lower arm */
static void Servo_OC4_Ticks(uint16_t t);   /* bucket arm (ticks) */

/*=========================== SERVO SLEW (SLOW MOTION) ============================*/
/* current commanded positions (what I last wrote to the OC regs) */
static uint16_t CurPushArmUs       = US_PUSH_ARM_UP;             /* OC5 */
static uint16_t CurBucketBottomUs  = US_BUCKET_STOP;             /* OC3 */
static uint16_t CurBucketArmTicks  = BUCKET_ARM_INIT_TICKS;      /* OC4 */

/* targets + per-update step sizes */
static uint16_t TargetPushArmUs       = US_PUSH_ARM_UP;
static uint16_t TargetBucketBottomUs  = US_BUCKET_STOP;
static uint16_t TargetBucketArmTicks  = BUCKET_ARM_INIT_TICKS;

static uint16_t StepPushArmUs         = 1u;
static uint16_t StepBucketBottomUs    = 1u;
static uint16_t StepBucketArmTicks    = 1u;

static bool SlewPushArmActive      = false;
static bool SlewBucketBottomActive = false;
static bool SlewBucketArmActive    = false;

static uint16_t AbsDiffU16(uint16_t a, uint16_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

static uint16_t ComputeStepU16(uint16_t cur, uint16_t tgt, uint16_t durationMs)
{
  /* how many updates during the move */
  uint32_t nSteps = (durationMs + SERVO_UPDATE_MS - 1u) / SERVO_UPDATE_MS;
  if (nSteps < 1u) nSteps = 1u;

  uint32_t diff = AbsDiffU16(cur, tgt);

  /* ceil(diff / nSteps), never 0 */
  uint32_t step = (diff + nSteps - 1u) / nSteps;
  if (step < 1u) step = 1u;

  return (uint16_t)step;
}

static void StartServoTimerIfNeeded(void)
{
  if (SlewPushArmActive || SlewBucketBottomActive || SlewBucketArmActive)
  {
    ES_Timer_InitTimer(SERVO_TIMER, SERVO_UPDATE_MS);
  }
}

static void StopServoTimerIfDone(void)
{
  if (!SlewPushArmActive && !SlewBucketBottomActive && !SlewBucketArmActive)
  {
    ES_Timer_StopTimer(SERVO_TIMER);
  }
}

/* start a smooth move for the push arm (OC5, positional servo) */
static void PushArmMoveTo(uint16_t targetUs, uint16_t durationMs)
{
  TargetPushArmUs   = targetUs;
  StepPushArmUs     = ComputeStepU16(CurPushArmUs, TargetPushArmUs, durationMs);
  SlewPushArmActive = true;
  StartServoTimerIfNeeded();
}

/* start a smooth move for the bucket arm (OC4, positional servo, ticks) */
static void BucketArmMoveToTicks(uint16_t targetTicks, uint16_t durationMs)
{
  TargetBucketArmTicks = targetTicks;
  StepBucketArmTicks   = ComputeStepU16(CurBucketArmTicks, TargetBucketArmTicks, durationMs);
  SlewBucketArmActive  = true;
  StartServoTimerIfNeeded();
}

/* optional: smooth the continuous rotation command too so it doesn't slam */
static void BucketBottomMoveTo(uint16_t targetUs, uint16_t durationMs)
{
  TargetBucketBottomUs   = targetUs;
  StepBucketBottomUs     = ComputeStepU16(CurBucketBottomUs, TargetBucketBottomUs, durationMs);
  SlewBucketBottomActive = true;
  StartServoTimerIfNeeded();
}

/* update all active slews; called every SERVO_UPDATE_MS */
static void ServoSlewUpdate(void)
{
  /* ---------- push arm OC5 ---------- */
  if (SlewPushArmActive)
  {
    if (CurPushArmUs == TargetPushArmUs)
    {
      SlewPushArmActive = false;
    }
    else if (CurPushArmUs < TargetPushArmUs)
    {
      uint16_t next = (uint16_t)(CurPushArmUs + StepPushArmUs);
      CurPushArmUs = (next > TargetPushArmUs) ? TargetPushArmUs : next;
      Servo_OC5(CurPushArmUs);
    }
    else
    {
      uint16_t next = (CurPushArmUs > StepPushArmUs) ? (uint16_t)(CurPushArmUs - StepPushArmUs) : 0u;
      CurPushArmUs = (next < TargetPushArmUs) ? TargetPushArmUs : next;
      Servo_OC5(CurPushArmUs);
    }
  }

  /* ---------- bucket arm OC4 ---------- */
  if (SlewBucketArmActive)
  {
    if (CurBucketArmTicks == TargetBucketArmTicks)
    {
      SlewBucketArmActive = false;
    }
    else if (CurBucketArmTicks < TargetBucketArmTicks)
    {
      uint16_t next = (uint16_t)(CurBucketArmTicks + StepBucketArmTicks);
      CurBucketArmTicks = (next > TargetBucketArmTicks) ? TargetBucketArmTicks : next;
      Servo_OC4_Ticks(CurBucketArmTicks);
    }
    else
    {
      uint16_t next = (CurBucketArmTicks > StepBucketArmTicks) ? (uint16_t)(CurBucketArmTicks - StepBucketArmTicks) : 0u;
      CurBucketArmTicks = (next < TargetBucketArmTicks) ? TargetBucketArmTicks : next;
      Servo_OC4_Ticks(CurBucketArmTicks);
    }
  }

  /* ---------- bucket bottom OC3 ---------- */
  if (SlewBucketBottomActive)
  {
    if (CurBucketBottomUs == TargetBucketBottomUs)
    {
      SlewBucketBottomActive = false;
    }
    else if (CurBucketBottomUs < TargetBucketBottomUs)
    {
      uint16_t next = (uint16_t)(CurBucketBottomUs + StepBucketBottomUs);
      CurBucketBottomUs = (next > TargetBucketBottomUs) ? TargetBucketBottomUs : next;
      Servo_OC3(CurBucketBottomUs);
    }
    else
    {
      uint16_t next = (CurBucketBottomUs > StepBucketBottomUs) ? (uint16_t)(CurBucketBottomUs - StepBucketBottomUs) : 0u;
      CurBucketBottomUs = (next < TargetBucketBottomUs) ? TargetBucketBottomUs : next;
      Servo_OC3(CurBucketBottomUs);
    }
  }

  StopServoTimerIfDone();
}

/*====================== ACTION HELPERS =====================*/
/* same action names as before; now they do smooth moves */
static void PushArmDown(void) { PushArmMoveTo(US_PUSH_ARM_DOWN, T_PUSH_ARM_DOWN_MS); }
static void PushArmUp(void)   { PushArmMoveTo(US_PUSH_ARM_UP,   T_PUSH_ARM_UP_MS);   }

/* continuous rotation servo: ramp a little so it doesn't jerk */
// static void BucketRotateStart(void) { BucketBottomMoveTo(US_BUCKET_ROTATE_CW, 120u); }
// static void BucketRotateStop(void)  { BucketBottomMoveTo(US_BUCKET_STOP,      120u); }

static void BucketRotateStart(void){ Servo_OC3(US_BUCKET_ROTATE_CW);} 
static void BucketRotateStop(void){  Servo_OC3(US_BUCKET_STOP);}   

static void BucketArmInit(void)     { BucketArmMoveToTicks(BUCKET_ARM_INIT_TICKS,     T_BUCKET_ARM_RETURN_MS); }
static void BucketArmDispense(void) { BucketArmMoveToTicks(BUCKET_ARM_DISPENSE_TICKS, T_BUCKET_ARM_MOVE_MS);   }

/*====================== SPI HELPERS =====================*/
static void RequestCmd(uint8_t cmdByte)
{
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*======================================================================
  FLAG SERVO (RB3) MS18-F analog positional servo

  Implementation:
    - Timer4 ISR generates continuous 50Hz pulses on RB3 as GPIO
    - ES_INDICATE_SIDE updates the pulse width; servo holds position
======================================================================*/

/* pulse widths (us) */
#define US_FLAG_MIN            900u // for MS18 was 600u
#define US_FLAG_MAX            2100u //for MS18 was 2500u
#define US_FLAG_CENTER         ((US_FLAG_MIN + US_FLAG_MAX)/2u)

#define US_FLAG_BLUE           US_FLAG_MAX
#define US_FLAG_GREEN          US_FLAG_MIN

#define FLAG_PERIOD_US         20000u  /* 50Hz */

/* RB3 GPIO */
#define FLAG_TRIS              TRISBbits.TRISB3
#define FLAG_LAT               LATBbits.LATB3
#define FLAG_ANSEL             ANSELBbits.ANSB3

/* PBCLK=20MHz, Timer4 prescale 1:8 => tick = 0.4us => 2.5 ticks/us */
#define T4_PRESCALE_BITS       0b11u   /* 11 = 1:8 */
#define TICKS_FROM_US(us)      ((uint16_t)(((uint32_t)(us) * 25u) / 10u))

typedef enum { FLAG_PHASE_PULSE = 0, FLAG_PHASE_REST = 1 } FlagPhase_t;
static volatile FlagPhase_t FlagPhase = FLAG_PHASE_PULSE;
static volatile uint16_t FlagPulseUs  = US_FLAG_CENTER;
static bool FlagInitDone = false;

static void InitFlagServoRB3(void);
static void FlagSetPulseUs(uint16_t us);

static void InitFlagServoRB3(void)
{
  if (FlagInitDone) return;

  /* detach PPS mapping from RB3 */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0b0000;
#endif

  FLAG_ANSEL = 0;
  FLAG_TRIS  = 0;
  FLAG_LAT   = 0;

  /* Timer4 setup */
  T4CON = 0;
  T4CONbits.TCS   = 0;
  T4CONbits.TCKPS = T4_PRESCALE_BITS;
  TMR4 = 0;

  /* Interrupt priority */
  IPC4bits.T4IP = 4;
  IPC4bits.T4IS = 0;

  IFS0CLR = _IFS0_T4IF_MASK;
  IEC0SET = _IEC0_T4IE_MASK;

  /* kick ISR */
  FlagPhase = FLAG_PHASE_PULSE;
  PR4 = 1;
  TMR4 = 0;

  T4CONbits.ON = 1;
  FlagInitDone = true;
}

static void FlagSetPulseUs(uint16_t us)
{
  __builtin_disable_interrupts();
  FlagPulseUs = us;
  __builtin_enable_interrupts();
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) T4Handler(void)
{
  IFS0CLR = _IFS0_T4IF_MASK;

  if (FlagPhase == FLAG_PHASE_PULSE)
  {
    FLAG_LAT = 1;
    FlagPhase = FLAG_PHASE_REST;

    TMR4 = 0;
    PR4  = TICKS_FROM_US(FlagPulseUs);
  }
  else
  {
    FLAG_LAT = 0;
    FlagPhase = FLAG_PHASE_PULSE;

    uint16_t restUs = (FlagPulseUs >= FLAG_PERIOD_US) ? 1u
                    : (uint16_t)(FLAG_PERIOD_US - FlagPulseUs);

    TMR4 = 0;
    PR4  = TICKS_FROM_US(restUs);
  }
}

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  InitFlagServoRB3();
  FlagSetPulseUs(US_FLAG_CENTER);

  /* safe startup poses (and sync my "current" vars to match what I write) */
  CurBucketBottomUs = US_BUCKET_STOP;
  Servo_OC3(CurBucketBottomUs);

  CurPushArmUs = US_PUSH_ARM_UP;
  Servo_OC5(CurPushArmUs);

  CurBucketArmTicks = BUCKET_ARM_INIT_TICKS;
  Servo_OC4_Ticks(CurBucketArmTicks);

  /* clear any leftover slews */
  SlewPushArmActive = false;
  SlewBucketBottomActive = false;
  SlewBucketArmActive = false;
  ES_Timer_StopTimer(SERVO_TIMER);

  CurState = DISP_IDLE;
  ES_Timer_StopTimer(DISPENSE_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, e);
}

bool PostDispenseService(ES_Event_t e)
{
  return ES_PostToService(MyPriority, e);
}

/*=========================== STATE MACHINE ============================*/
ES_Event_t RunDispenseService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  /* side indication can happen anytime */
  if (ThisEvent.EventType == ES_INDICATE_SIDE)
  {
    Field_t side = (Field_t)ThisEvent.EventParam;
    DB_printf("Side indicated! Moving flag. side=%d\r\n", side);

    if (side == FIELD_BLUE) {
      FlagSetPulseUs(US_FLAG_BLUE);
      DB_printf("set arm to blue \r\n");
    } else if (side == FIELD_GREEN) {
      FlagSetPulseUs(US_FLAG_GREEN);
      DB_printf("set arm to green \r\n");
    } else {
      FlagSetPulseUs(US_FLAG_CENTER);
    }

    return ReturnEvent;
  }

  /* servo slew updates happen "in the background" using SERVO_TIMER */
  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SERVO_TIMER))
  {
    ServoSlewUpdate();
    StartServoTimerIfNeeded(); /* keep it periodic while anything is still moving */
    return ReturnEvent;
  }

  switch (CurState)
  {
    case DISP_IDLE:
      if (ThisEvent.EventType == ES_DISPENSE_START)
      {
        DB_printf("Dispense start\r\n");

        /* lower arm comes down first while bucket arm stays initialized */
        BucketArmInit();
        PushArmDown();

#ifdef CMD_DISPENSE_START
        RequestCmd(CMD_DISPENSE_START);
#endif

        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_PUSH_ARM_DOWN_MS);
      }
      break;

    case DISP_PUSH_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_ARM_CLEAR_DELAY;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_AFTER_ARM_DOWN_DELAY_MS);
      }
      break;

    case DISP_ARM_CLEAR_DELAY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketArmDispense();
        CurState = DISP_BUCKET_ARM_TO_DISPENSE;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_BUCKET_ARM_MOVE_MS);
      }
      break;

    case DISP_BUCKET_ARM_TO_DISPENSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_BUCKET_ARM_SETTLE;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_BUCKET_ARM_SETTLE_MS);
      }
      break;

    case DISP_BUCKET_ARM_SETTLE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketRotateStart();
        CurState = DISP_ROTATE_BUCKET;

        if (FirstDispense)
        {
          ES_Timer_InitTimer(DISPENSE_TIMER, T_ROTATE_BUCKET_90_MS);
          FirstDispense = false;
        }
        else
        {
          ES_Timer_InitTimer(DISPENSE_TIMER, T_ROTATE_BUCKET_180_MS);
          FirstDispense = true;
        }
      }
      break;

    case DISP_ROTATE_BUCKET:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketRotateStop();
        CurState = DISP_WAIT_DROP;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        PushArmUp();
        CurState = DISP_PUSH_ARM_UP;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_PUSH_ARM_UP_MS);
      }
      break;

    case DISP_PUSH_ARM_UP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketArmInit();
        CurState = DISP_BUCKET_ARM_RETURN;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_BUCKET_ARM_RETURN_MS);
      }
      break;

    case DISP_BUCKET_ARM_RETURN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_DONE;
        ES_Timer_InitTimer(DISPENSE_TIMER, 10u);
      }
      break;

    case DISP_DONE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        DB_printf("Dispense complete\r\n");

#ifdef CMD_DISPENSE_DONE
        RequestCmd(CMD_DISPENSE_DONE);
#endif

        ES_Event_t done = { ES_DISPENSE_COMPLETE, 0 };
        ES_PostAll(done);

        CurState = DISP_IDLE;
      }
      break;

    default:
      CurState = DISP_IDLE;
      break;
  }

  return ReturnEvent;
}

/*=========================== PWM ============================*/
static void InitServoPWM(void)
{
  /*
    Servo PWM base:
      PBCLK=20MHz
      prescale=1:8 -> tick=0.4us -> 2.5 ticks/us
      PR2=49999 -> period = 50000 ticks = 20,000us = 50Hz
  */
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCKPS = 0b011u; /* 1:8 */
    TMR2 = 0;
    PR2 = 49999u;
    T2CONbits.ON = 1;
  }

  /* digital outputs */
  TRISBbits.TRISB6  = 0;
  TRISBbits.TRISB10 = 0;
  TRISAbits.TRISA4  = 0;

  /* OC3 bottom */
  OC3CON = 0; OC3R = 0; OC3RS = 0;
  OC3CONbits.OCTSEL = 0;
  OC3CONbits.OCM    = 0b110;
  OC3CONbits.ON     = 1;

  /* OC5 lower arm */
  OC5CON = 0; OC5R = 0; OC5RS = 0;
  OC5CONbits.OCTSEL = 0;
  OC5CONbits.OCM    = 0b110;
  OC5CONbits.ON     = 1;

  /* OC4 bucket arm */
  OC4CON = 0; OC4R = 0; OC4RS = 0;
  OC4CONbits.OCTSEL = 0;
  OC4CONbits.OCM    = 0b110;
  OC4CONbits.ON     = 1;

  /* PPS mapping */
  RPB6Rbits.RPB6R   = 0b0110; /* OC5 -> RB6 */
  RPB10Rbits.RPB10R = 0b0101; /* OC3 -> RB10 */
  RPA4Rbits.RPA4R   = 0b0101; /* OC4 -> RA4 */

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us * 25u) / 10u); /* 2.5 ticks/us */
}

static void Servo_OC3(uint16_t us){ OC3RS = UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS = UsToOCrs(us); }
static void Servo_OC4_Ticks(uint16_t t){ OC4RS = t; }