/****************************************************************************
 Module
   DispenseService.c  (MIN FIX: stop bucket following collector arm)

 Summary
   Same logic as your rewritten version:
     - Go READY on ES_DISPENSE_START
     - WAIT until ES_COLLECT_DONE
     - Then move to DISPENSE and do 90deg + 180deg timed dumps
   Fixes:
     (1) PPS mapping conflict: detach first, then map OC3/OC5 explicitly
     (2) Proper OC init: set OCxR + OCxRS
     (3) READY and DISPENSE pulse widths must be different
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

#define BUCKET_RAMP_TIMER      14u     /* must route to PostDispenseService */
#define BUCKET_RAMP_TICK_MS    10u     /* 10ms update */
#define BUCKET_RAMP_STEP_US    5u

/*=========================== TIMING (TUNE) ============================*/
#define T_READY_SETTLE_MS      300u
#define T_SWING_MOVE_MS        600u

#define T_DUMP_90_MS           400u
#define T_DUMP_180_MS          800u
#define T_DROP_WAIT_MS         500u

/*=========================== SERVO PULSEWIDTHS ============================*/
/*
  IMPORTANT FIX:
  READY and DISPENSE MUST be different.
  Tune these to your mechanism.
*/
#define US_SWING_READY         2000u   /* collector side / ready */
#define US_SWING_DISPENSE      1200u   /* dispense side */

/* bucket bottom (continuous rotation) */
#define US_BOTTOM_STOP         1500u
#define US_BOTTOM_ROTATE_CW    1700u
#define US_BOTTOM_ROTATE_CCW   1300u

#define BOTTOM_DUMP_US         US_BOTTOM_ROTATE_CW

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE = 0,

  DISP_GO_READY,
  DISP_WAIT_COLLECT_DONE,

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

static void Servo_OC3(uint16_t us); /* bottom */
static void Servo_OC5(uint16_t us); /* swing */

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
  Servo_OC5(us);
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

  Servo_OC5(SwingCurUs);

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

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == BUCKET_RAMP_TIMER)) {
    Swing_RampTick();
    return ReturnEvent;
  }

  switch (CurState)
  {
    case DISP_IDLE:
      if (ThisEvent.EventType == ES_DISPENSE_START)
      {
        BottomStop();
        SwingToReady();
        CurState = DISP_GO_READY;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_READY_SETTLE_MS);
      }
      break;

    case DISP_GO_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_WAIT_COLLECT_DONE; /* HOLD HERE */
      }
      break;

    case DISP_WAIT_COLLECT_DONE:
      if (ThisEvent.EventType == ES_COLLECT_DONE)
      {
        SwingToDispense();
        CurState = DISP_MOVE_TO_DISPENSE;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_SWING_MOVE_MS);
      }
      break;

    case DISP_MOVE_TO_DISPENSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
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
  /* Shared Timer2 servo base — init ONLY if OFF */
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS; /* 1:8 */
    TMR2 = 0;
    PR2 = (uint16_t)SERVO_PR2_VALUE;
    T2CONbits.ON = 1;
  }

  /* pins output */
  TRISBbits.TRISB6  = 0;
  TRISBbits.TRISB10 = 0;

  /* ===== CRITICAL FIX: detach PPS first ===== */
  RPB6Rbits.RPB6R   = 0;
  RPB10Rbits.RPB10R = 0;

  /* OC3 (bottom) PWM on Timer2 — init OCxR + OCxRS */
  OC3CON = 0;
  OC3CONbits.OCTSEL = 0;
  OC3R  = UsToOCrs(US_BOTTOM_STOP);
  OC3RS = UsToOCrs(US_BOTTOM_STOP);
  OC3CONbits.OCM = 0b110;
  OC3CONbits.ON  = 1;

  /* OC5 (swing) PWM on Timer2 — init OCxR + OCxRS */
  OC5CON = 0;
  OC5CONbits.OCTSEL = 0;
  OC5R  = UsToOCrs(US_SWING_READY);
  OC5RS = UsToOCrs(US_SWING_READY);
  OC5CONbits.OCM = 0b110;
  OC5CONbits.ON  = 1;

  /*
    ===== PPS CODES (YOU MAY NEED TO CHANGE THESE) =====
    Common PIC32MX:
      OC3 = 0b0111
      OC5 = 0b1001
    If your bucket still mirrors the collector arm, your PPS codes are wrong.
  */
#ifndef PPS_OC3_CODE
#define PPS_OC3_CODE 0b0111
#endif
#ifndef PPS_OC5_CODE
#define PPS_OC5_CODE 0b1001
#endif

  RPB10Rbits.RPB10R = PPS_OC3_CODE; /* OC3 -> RB10 */
  RPB6Rbits.RPB6R   = PPS_OC5_CODE; /* OC5 -> RB6  */

  ServoInitDone = true;

  DB_printf("Dispense PWM init ok: Timer2=%uHz PR2=%u OC3code=%u OC5code=%u\r\n",
            (unsigned)SERVO_HZ, (unsigned)PR2,
            (unsigned)PPS_OC3_CODE, (unsigned)PPS_OC5_CODE);
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* PBCLK=20MHz, prescale=8 => 2.5 MHz => 1us = 2.5 ticks */
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void Servo_OC3(uint16_t us) { OC3RS = UsToOCrs(us); }
static void Servo_OC5(uint16_t us) { OC5RS = UsToOCrs(us); }