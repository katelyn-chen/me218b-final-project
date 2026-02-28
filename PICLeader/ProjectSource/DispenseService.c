/****************************************************************************
 Module
   DispenseService.c

 Summary
   Dispense sequence using two arm servos and rotating bucket bottom.

   Sequence:
     1) Lower push-down arm
     2) Small backup delay
     3) Move bucket from collector side → dispense side
     4) Rotate bucket bottom +90 degrees
     5) Wait for balls to fall
     6) Return bucket to collector side
     7) Raise push arm while navigating away
****************************************************************************/

#include "DispenseService.h"
#include "SPILeaderService.h"

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*============================== TIMING ==============================*/
#define T_PUSH_ARM_DOWN_MS     950u
#define T_BACKUP_DELAY_MS      100u   /* robot backing slightly */
#define T_BUCKET_MOVE_MS       1900u
#define T_ROTATE_BUCKET_MS     900u
#define T_DISPENSE_WAIT_MS     1200u
#define T_RETURN_MS            800u
#define T_PUSH_ARM_UP_MS       1500u

/*=========================== SERVO POSITIONS ============================*/
/* OC5 : push-down arm */
#define US_PUSH_ARM_UP        1200u
#define US_PUSH_ARM_DOWN      2000u

/* OC4 : bucket swing arm (collector <-> dispense) */
#define US_BUCKET_COLLECT     900u
#define US_BUCKET_DISPENSE    4500u

/* OC3 : continuous rotation bucket bottom */
#define US_BUCKET_STOP        1500u
#define US_BUCKET_ROTATE_CW   1700u

#define BUCKET_ARM_MIN_TICKS  1400
#define BUCKET_ARM_MAX_TICKS  3400

#define BUCKET_COLLECT_TICKS  1540

/*============================== TIMER ==============================*/
#ifndef DISPENSE_TIMER
#define DISPENSE_TIMER         2u
#endif

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE,
  DISP_PUSH_ARM_DOWN,
  DISP_BACKUP_DELAY,
  DISP_MOVE_BUCKET,
  DISP_ROTATE_BUCKET,
  DISP_WAIT_DROP,
  DISP_RETURN_BUCKET,
  DISP_PUSH_ARM_UP,
  DISP_DONE
} DispState_t;

static DispState_t CurState = DISP_IDLE;
static uint8_t MyPriority;

/* track cumulative bucket rotation */
static uint8_t BucketQuarterTurns = 0;

/*====================== PWM HELPERS =====================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);

static void Servo_OC3(uint16_t us); // bucket bottom
static void Servo_OC4(uint16_t us); // bucket arm 
static void Servo_OC5(uint16_t us); // lower arm

/*====================== ACTION HELPERS =====================*/
static void PushArmDown(void){ Servo_OC5(US_PUSH_ARM_DOWN); }
static void PushArmUp(void){ Servo_OC5(US_PUSH_ARM_UP); }

static void BucketToDispense(void){ Servo_OC4(US_BUCKET_DISPENSE); }
static void BucketToCollect(void){ Servo_OC4(BUCKET_COLLECT_TICKS); }

static void BucketRotateStart(void){ Servo_OC3(US_BUCKET_ROTATE_CW); }
static void BucketRotateStop(void){ Servo_OC3(US_BUCKET_STOP); }

/*====================== SPI HELPERS =====================*/
static void RequestCmd(uint8_t cmdByte)
{
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*======================================================================
  FLAG SERVO (RB3) — SOFTWARE SERVO OUTPUT (50 Hz) HOLD POSITION
  - Uses Timer4 ISR to generate continuous pulses on RB3
  - Reacts to ES_INDICATE_SIDE by changing pulse width
======================================================================*/

/* pulse widths (tune as needed after testing angles) */
#define US_FLAG_BLUE          2000u  /* +90 */
#define US_FLAG_GREEN         1000u  /* -90 */
#define US_FLAG_CENTER        1500u

#define SERVO_PERIOD_US       20000u  /* 20 ms */

#define FLAG_TRIS             TRISBbits.TRISB3
#define FLAG_LAT              LATBbits.LATB3
#define FLAG_ANSEL            ANSELBbits.ANSB3

/* PBCLK = 20MHz, Timer4 prescale 1:8 => 2.5MHz => 2.5 ticks/us */
#define T4_PRESCALE_BITS      0b11u   /* 1:8 (PIC32: 11=1:8) */
#define TICKS_FOR_US(us)      ((uint16_t)(((uint32_t)(us) * 25u) / 10u)) /* us * 2.5 */

typedef enum { FLAG_PHASE_START = 0, FLAG_PHASE_END = 1 } FlagPhase_t;
static volatile FlagPhase_t FlagPhase = FLAG_PHASE_START;
static volatile uint16_t FlagPulseUs = US_FLAG_CENTER;
static bool FlagInitDone = false;

static void InitFlagServoRB3(void);
static void FlagSetPulseUs(uint16_t us);

static void InitFlagServoRB3(void)
{
  if (FlagInitDone) return;

  /* Make sure RB3 is digital GPIO */
  FLAG_ANSEL = 0;
  FLAG_TRIS  = 0;
  FLAG_LAT   = 0;

  /* IMPORTANT: ensure RB3 isn't driven by PPS output (OC/etc.) */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0;   /* detach PPS output from RB3 */
#endif

  /* enable multi-vector interrupts (safe even if already enabled) */
  INTCONbits.MVEC = 1;

  /* Timer4 setup */
  T4CON = 0;
  T4CONbits.TCS   = 0;
  T4CONbits.TCKPS = T4_PRESCALE_BITS;
  TMR4 = 0;

  /* Timer4 interrupt priority */
  IPC4bits.T4IP = 4;
  IPC4bits.T4IS = 0;

  IFS0CLR = _IFS0_T4IF_MASK;
  IEC0SET = _IEC0_T4IE_MASK;

  /* schedule first interrupt quickly */
  FlagPhase = FLAG_PHASE_START;
  PR4 = 1;
  TMR4 = 0;

  T4CONbits.ON = 1;

  FlagInitDone = true;
}

static void FlagSetPulseUs(uint16_t us)
{
  /* clamp */
  if (us < 800u)  us = 800u;
  if (us > 2200u) us = 2200u;

  /* avoid ISR reading mid-update */
  __builtin_disable_interrupts();
  FlagPulseUs = us;
  __builtin_enable_interrupts();
}

/* Timer4 ISR: toggles RB3 to generate servo pulses continuously */
void __ISR(_TIMER_4_VECTOR, IPL6SOFT) T4Handler(void)
{
  IFS0CLR = _IFS0_T4IF_MASK;

  if (FlagPhase == FLAG_PHASE_START)
  {
    /* start pulse */
    FLAG_LAT = 1;
    FlagPhase = FLAG_PHASE_END;

    TMR4 = 0;
    PR4  = TICKS_FOR_US(FlagPulseUs);
  }
  else
  {
    /* end pulse, wait remainder of period */
    FLAG_LAT = 0;
    FlagPhase = FLAG_PHASE_START;

    uint16_t remainUs = (FlagPulseUs >= SERVO_PERIOD_US) ? 1u
                      : (uint16_t)(SERVO_PERIOD_US - FlagPulseUs);

    TMR4 = 0;
    PR4  = TICKS_FOR_US(remainUs);
  }
}

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  /* init flag servo output and hold center by default */
  InitFlagServoRB3();
  FlagSetPulseUs(US_FLAG_CENTER);

  BucketQuarterTurns = 0u;
//  BucketToCollect();
  BucketRotateStop();
  PushArmUp();

  ES_Timer_StopTimer(DISPENSE_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { DISP_IDLE,0 };
  return ES_PostToService(MyPriority,e);
}

bool PostDispenseService(ES_Event_t e)
{
  return ES_PostToService(MyPriority,e);
}

/*=========================== STATE MACHINE ============================*/
ES_Event_t RunDispenseService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = {ES_NO_EVENT,0};

  switch(CurState)
  {
    case DISP_IDLE:
      if(ThisEvent.EventType == ES_DISPENSE_START)
      {
        DB_printf("Dispense start\r\n");
        PushArmDown();

#ifdef CMD_DISPENSE_START
        RequestCmd(CMD_DISPENSE_START);
#endif

        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_DOWN_MS);
      }

      /* SIDE INDICATOR EVENT: set flag position and HOLD (Timer4 keeps pulsing) */
      if(ThisEvent.EventType == ES_INDICATE_SIDE)
      {
        Field_t side = (Field_t)ThisEvent.EventParam;
        DB_printf("Indicating side! %d\r\n", side);

        if (side == FIELD_BLUE) {
          FlagSetPulseUs(US_FLAG_BLUE);
        } else if (side == FIELD_GREEN) {
          FlagSetPulseUs(US_FLAG_GREEN);
        } else {
          FlagSetPulseUs(US_FLAG_CENTER);
        }
      }      
      if(ThisEvent.EventType == ES_WAIT_BALL) {
        BucketToCollect(); // move bucket to collect position and stop there
        DB_printf("bucket moved to collect \n");
      }
      break; 

    case DISP_PUSH_ARM_DOWN:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        CurState = DISP_BACKUP_DELAY;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_BACKUP_DELAY_MS);
      }
      break;

    case DISP_BACKUP_DELAY:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        BucketToDispense();
        CurState = DISP_MOVE_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_BUCKET_MOVE_MS);
      }
      break;

    case DISP_MOVE_BUCKET:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        BucketRotateStart();
        CurState = DISP_ROTATE_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_ROTATE_BUCKET_MS);
      }
      break;

    case DISP_ROTATE_BUCKET:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        BucketRotateStop();
        BucketQuarterTurns++;
        CurState = DISP_WAIT_DROP;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        BucketToCollect();
        CurState = DISP_RETURN_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_RETURN_MS);
      }
      break;

    case DISP_RETURN_BUCKET:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        PushArmUp();
        CurState = DISP_PUSH_ARM_UP;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_UP_MS);
      }
      break;

    case DISP_PUSH_ARM_UP:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        CurState = DISP_DONE;
      }
      break;

    case DISP_DONE:
    {
      DB_printf("Dispense complete\r\n");

#ifdef CMD_DISPENSE_DONE
      RequestCmd(CMD_DISPENSE_DONE);
#endif

      ES_Event_t done = {ES_DISPENSE_COMPLETE,0};
      ES_PostAll(done);

      CurState = DISP_IDLE;
      break;
    }

    default:
      CurState = DISP_IDLE;
      break;
  }

  return ReturnEvent;
}

/*=========================== PWM ============================*/
static void InitServoPWM(void)
{
    
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCKPS = 0b011u;
    TMR2 = 0;
    PR2 = 49999u; /* 20ms at PBCLK=20MHz prescale 1:8 */
    T2CONbits.ON = 1;
  }
  ANSELBbits.ANSB3 = 0;

  TRISBbits.TRISB3 = 0;
  TRISBbits.TRISB6 = 0;
  /* ensure OCs use Timer2 */
  OC3CON=0; OC3R=0; OC3RS=0; OC3CONbits.OCTSEL=0; OC3CONbits.OCM=0b110; OC3CONbits.ON=1;
  OC4CON=0; OC4R=0; OC4RS=0; OC4CONbits.OCTSEL=0; OC4CONbits.OCM=0b110; OC4CONbits.ON=1;
  OC5CON=0; OC5R=0; OC5RS=0; OC5CONbits.OCTSEL=0; OC5CONbits.OCM=0b110; OC5CONbits.ON=1;
  
  RPB3Rbits.RPB3R = 0b0101;  
  RPA4Rbits.RPA4R = 0b0101;  
  RPB6Rbits.RPB6R = 0b0110; 
  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us*25u)/10u);
}

static inline uint16_t ClampArm(uint16_t v)
{
  if (v < BUCKET_ARM_MIN_TICKS) return BUCKET_ARM_MIN_TICKS;
  if (v > BUCKET_ARM_MAX_TICKS) return BUCKET_ARM_MAX_TICKS;
  return v;
}

static void Servo_OC3(uint16_t us){ OC3RS=UsToOCrs(us); }
//static void Servo_OC4(uint16_t us){ OC4RS=UsToOCrs(us); }
static void Servo_OC4(uint16_t ticks) {OC1RS = ClampArm(ticks);}
static void Servo_OC5(uint16_t us){ OC5RS=UsToOCrs(us); }