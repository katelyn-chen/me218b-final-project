/****************************************************************************
 Module
   DispenseService.c  (REWRITE: wait-for-collect then dispense in 2 dumps)

 Summary
   Bucket dispense sequence that COEXISTS with CollectService.c.

   Hardware (recommended to avoid conflicts):
     - Timer2: shared 50 Hz servo timebase (DO NOT stomp if already running)
     - OC5 (RB6): bucket SWING arm  (collector side <-> dispense side)
     - OC3 (RB10): bucket BOTTOM servo (continuous rotation or motorized)

   Desired logic:
     A) On ES_DISPENSE_START:
        1) Move swing arm to READY (collector side) and HOLD there
        2) Wait until CollectService finishes all 6 balls (ES_COLLECT_DONE)
     B) On ES_COLLECT_DONE:
        3) Move swing arm to DISPENSE side
        4) Rotate bottom for "first half" dump (90 deg equivalent time)
        5) Rotate bottom for "second half" dump (180 deg additional time)
        6) Return swing arm to READY (optional) and done

 Notes
   - Uses DISPENSE_TIMER for sequencing.
   - Uses BUCKET_RAMP_TIMER (ES timer 14 by default) for slow ramping of swing arm.
   - REQUIRES in ES_Configure.h:
       #define TIMER14_RESP_FUNC PostDispenseService
****************************************************************************/

#include "DispenseService.h"
#include "SPILeaderService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

#define PBCLK_HZ               20000000u

/*=========================== TIMER2 SERVO BASE ============================*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u
#define SERVO_HZ                    50u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/*============================== TIMERS ==============================*/
#ifndef DISPENSE_TIMER
#define DISPENSE_TIMER         2u
#endif

/* Swing-arm ramp timer (separate from DISPENSE_TIMER) */
#define BUCKET_RAMP_TIMER      14u     /* must route to PostDispenseService */
#define BUCKET_RAMP_TICK_MS    10u     /* 10ms update like CollectService */
#define BUCKET_RAMP_STEP_US    5u      /* smaller = slower */

/*=========================== TIMING (TUNE) ============================*/
/* How long to wait after commanding swing arm moves */
#define T_READY_SETTLE_MS      300u
#define T_SWING_MOVE_MS        600u

/* Bottom "dump" timing (you MUST tune these to your hardware) */
#define T_DUMP_90_MS           400u    /* time to rotate ~90 degrees */
#define T_DUMP_180_MS          800u    /* time to rotate ~180 degrees */
#define T_DROP_WAIT_MS         500u    /* wait for balls to fall */

/*=========================== SERVO PULSEWIDTHS ============================*/
/* OC5: bucket swing arm (collector side <-> dispense side) */
#define US_SWING_READY         1500u   /* collector side / ready to receive balls (TUNE) */
#define US_SWING_DISPENSE      1500u   /* dispense side (TUNE) */

/* OC3: bucket bottom (continuous rotation style) */
#define US_BOTTOM_STOP         1500u
#define US_BOTTOM_ROTATE_CW    1700u   /* pick direction that dumps correctly */
#define US_BOTTOM_ROTATE_CCW   1300u   /* optional if you need reverse */

/* Choose which direction dumps out */
#define BOTTOM_DUMP_US         US_BOTTOM_ROTATE_CW

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE = 0,

  DISP_GO_READY,           /* move swing arm to ready and open bottom stop */
  DISP_WAIT_COLLECT_DONE,  /* hold ready until collect done event arrives */

  DISP_MOVE_TO_DISPENSE,
  DISP_DUMP_90,
  DISP_WAIT_DROP1,
  DISP_DUMP_180,
  DISP_WAIT_DROP2,

  DISP_RETURN_READY,
  DISP_DONE
} DispState_t;

static DispState_t CurState = DISP_IDLE;
static uint8_t MyPriority;

/*====================== PWM HELPERS =====================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);

static void Servo_OC3(uint16_t us); /* bucket bottom */
static void Servo_OC4(uint16_t us); /* swing arm */

/*====================== SWING ARM RAMP ENGINE =====================*/
static uint16_t SwingCurUs    = US_SWING_READY;
static uint16_t SwingTargetUs = US_SWING_READY;
static bool     SwingRamping  = false;

static void Swing_SetImmediate(uint16_t us);
static void Swing_GotoSlow(uint16_t targetUs);
static void Swing_RampTick(void);

static void Swing_SetImmediate(uint16_t us)
{
  SwingCurUs = us;
  Servo_OC4(us);
}

static void Swing_GotoSlow(uint16_t targetUs)
{
  SwingTargetUs = targetUs;

  if (SwingCurUs == SwingTargetUs) {
    SwingRamping = false;
    ES_Timer_StopTimer(BUCKET_RAMP_TIMER);
    return;
  }

  SwingRamping = true;
  ES_Timer_InitTimer(BUCKET_RAMP_TIMER, BUCKET_RAMP_TICK_MS);
}

static void Swing_RampTick(void)
{
  if (!SwingRamping) return;

  int32_t err = (int32_t)SwingTargetUs - (int32_t)SwingCurUs;
  if (err == 0) {
    SwingRamping = false;
    ES_Timer_StopTimer(BUCKET_RAMP_TIMER);
    return;
  }

  int32_t step = (err > 0) ? (int32_t)BUCKET_RAMP_STEP_US : -(int32_t)BUCKET_RAMP_STEP_US;

  if ((err > 0 && step > err) || (err < 0 && step < err)) {
    SwingCurUs = SwingTargetUs;
  } else {
    SwingCurUs = (uint16_t)((int32_t)SwingCurUs + step);
  }

  Servo_OC4(SwingCurUs);

  if (SwingCurUs == SwingTargetUs) {
    SwingRamping = false;
    ES_Timer_StopTimer(BUCKET_RAMP_TIMER);
  } else {
    ES_Timer_InitTimer(BUCKET_RAMP_TIMER, BUCKET_RAMP_TICK_MS);
  }
}

/*====================== ACTION HELPERS =====================*/
static void BottomStop(void)      { Servo_OC3(US_BOTTOM_STOP); }
static void BottomDumpStart(void) { Servo_OC3(BOTTOM_DUMP_US); }

static void SwingToReady(void)    { Swing_GotoSlow(US_SWING_READY); }
static void SwingToDispense(void) { Swing_GotoSlow(US_SWING_DISPENSE); }

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  /* Start safe: bottom stopped, swing in ready */
  BottomStop();
  Swing_SetImmediate(US_SWING_READY);
  SwingTargetUs = US_SWING_READY;
  SwingRamping = false;

  ES_Timer_StopTimer(BUCKET_RAMP_TIMER);
  ES_Timer_StopTimer(DISPENSE_TIMER);

  CurState = DISP_IDLE;

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

  switch (CurState)
  {
    case DISP_IDLE:
      if (ThisEvent.EventType == ES_DISPENSE_START)
      {
        /* Step A1: move to ready position first */
        BottomStop();
        SwingToReady();
        CurState = DISP_GO_READY;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_READY_SETTLE_MS);
      }
      break;

    case DISP_GO_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        /* Step A2: wait here until collect is done */
        CurState = DISP_WAIT_COLLECT_DONE;
      }
      break;

    case DISP_WAIT_COLLECT_DONE:
      /* This is the key handshake: YOU MUST POST THIS EVENT when Collect finishes 6 balls */
      if (ThisEvent.EventType == ES_COLLECT_DONE)
      {
        /* Step B3: move to dispense side */
        SwingToDispense();
        CurState = DISP_MOVE_TO_DISPENSE;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_SWING_MOVE_MS);
      }
      break;

    case DISP_MOVE_TO_DISPENSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        /* Step B4: first half dump (90 deg worth of time) */
        BottomDumpStart();
        CurState = DISP_DUMP_90;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DUMP_90_MS);
      }
      break;

    case DISP_DUMP_90:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BottomStop();
        CurState = DISP_WAIT_DROP1;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DROP_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP1:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        /* Step B5: second half dump (180 deg more from where it was) */
        BottomDumpStart();
        CurState = DISP_DUMP_180;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DUMP_180_MS);
      }
      break;

    case DISP_DUMP_180:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BottomStop();
        CurState = DISP_WAIT_DROP2;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DROP_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP2:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        /* Step B6: return to ready (optional but recommended) */
        SwingToReady();
        CurState = DISP_RETURN_READY;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_SWING_MOVE_MS);
      }
      break;

    case DISP_RETURN_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_DONE;
      }
      break;

    case DISP_DONE:
    {
      /* Notify rest of system if you want */
      ES_Event_t done = { ES_DISPENSE_COMPLETE, 0 };
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

/*=========================== PWM INIT ============================*/
static void InitServoPWM(void)
{
  /* Use Timer2 as shared servo timebase — init ONLY if it is OFF */
  // if (T2CONbits.ON == 0u)
  // {
  //   T2CON = 0;
  //   T2CONbits.TCS = 0;
  //   T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS; /* 1:8 */
  //   TMR2 = 0;
  //   PR2 = (uint16_t)SERVO_PR2_VALUE;
  //   T2CONbits.ON = 1;
  // }

  /* OC3 (bottom) PWM on Timer2 */
  OC3CON = 0; OC3R = 0; OC3RS = 0;
  OC3CONbits.OCTSEL = 0;     /* Timer2 */
  OC3CONbits.OCM = 0b110;    /* PWM */
  OC3CONbits.ON = 1;

  /* OC5 (lower arm) PWM on Timer2 */
  OC5CON = 0; OC5R = 0; OC5RS = 0;
  OC5CONbits.OCTSEL = 0;     /* Timer2 */
  OC5CONbits.OCM = 0b110;    /* PWM */
  OC5CONbits.ON = 1;

  /* PPS mapping */
  TRISBbits.TRISB6  = 0;  /* RB6 output */
  TRISBbits.TRISB10 = 0;  /* RB10 output */
  RPB6Rbits.RPB6R   = 0b0110; /* OC5 -> RB6 (as your old code) */
  RPB10Rbits.RPB10R = 0b0101; /* OC3 -> RB10 (as your old code) */

  ServoInitDone = true;
  DB_printf("DispenseService: PWM init ok (Timer2 %u Hz), OC5 swing + OC3 bottom.\r\n",
            (unsigned)SERVO_HZ);
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* PBCLK=20MHz, prescale=8 => 2.5 MHz => 1us = 2.5 ticks */
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void Servo_OC3(uint16_t us) { OC3RS = UsToOCrs(us); }
static void Servo_OC4(uint16_t us) { OC4RS = UsToOCrs(us); }