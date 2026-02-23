/****************************************************************************
  Module
    DispenseService.c

  Summary
    - Dispenses balls from bucket in two stages
      1st dispense: gate opens halfway
      2nd dispense: gate opens fully
    - DispenseService keeps an internal toggle to pick which stage is next

  Notes
    - Uses bucket arm servo (OC5) and gate servo (OC3)
    - Servo PWM timebase is Timer2 (50 Hz)
****************************************************************************/

#include "DispenseService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ               20000000u

#ifndef DISPENSE_TIMER
#define DISPENSE_TIMER         DISPENSE_TIMER
#endif

/* timing (ms) */
#define T_ARM_DOWN_MS          500u
#define T_GATE_OPEN_MS         250u
#define T_DISPENSE_WAIT_MS     650u
#define T_GATE_CLOSE_MS        250u
#define T_ARM_UP_MS            500u

/*=========================== SERVO PWM HW ============================*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u
#define SERVO_HZ                    50u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/* bucket arm (OC5) */
#define US_BUCKET_ARM_UP            1200u
#define US_BUCKET_ARM_DOWN          2000u

/* gate servo (OC3) */
#define US_GATE_CLOSED              1100u
#define US_GATE_HALF_OPEN           1500u
#define US_GATE_FULL_OPEN           1900u

static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);
static void ServoSet_OC3_us(uint16_t us);
static void ServoSet_OC5_us(uint16_t us);

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE = 0,
  DISP_ARM_DOWN,
  DISP_GATE_OPEN,
  DISP_WAIT,
  DISP_GATE_CLOSE,
  DISP_ARM_UP,
  DISP_DONE
} DispState_t;

static uint8_t MyPriority;
static DispState_t CurState = DISP_IDLE;

/* internal toggle: false -> half open next, true -> full open next */
static bool NextIsFull = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void TransitionTo(DispState_t next, uint16_t tMs);

static void BucketArmDown(void);
static void BucketArmUp(void);
static void GateClose(void);
static void GateOpenHalf(void);
static void GateOpenFull(void);

/*=========================== PUBLIC API =============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = DISP_IDLE;
  NextIsFull = false;

  if (!ServoInitDone) InitServoPWM();

  GateClose();
  BucketArmUp();

  ES_Timer_StopTimer(DISPENSE_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, e);
}

bool PostDispenseService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunDispenseService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (CurState)
  {
    case DISP_IDLE:
      if (ThisEvent.EventType == ES_DISPENSE_START)
      {
        DB_printf("DispenseService: start (next=%s)\r\n",
                  NextIsFull ? "FULL" : "HALF");

        TransitionTo(DISP_ARM_DOWN, T_ARM_DOWN_MS);
      }
      break;

    case DISP_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        TransitionTo(DISP_GATE_OPEN, T_GATE_OPEN_MS);
      }
      break;

    case DISP_GATE_OPEN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        TransitionTo(DISP_WAIT, T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        TransitionTo(DISP_GATE_CLOSE, T_GATE_CLOSE_MS);
      }
      break;

    case DISP_GATE_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        TransitionTo(DISP_ARM_UP, T_ARM_UP_MS);
      }
      break;

    case DISP_ARM_UP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        TransitionTo(DISP_DONE, 0u);
      }
      break;

    case DISP_DONE:
    {
      DB_printf("DispenseService: done\r\n");

      ES_Event_t doneEvt = { ES_DISPENSE_DONE, 0 };
      ES_PostAll(doneEvt);

      /* toggle stage for next call */
      NextIsFull = !NextIsFull;

      TransitionTo(DISP_IDLE, 0u);
      break;
    }

    default:
      TransitionTo(DISP_IDLE, 0u);
      break;
  }

  return ReturnEvent;
}

/*=========================== TRANSITIONS ============================*/
static void TransitionTo(DispState_t next, uint16_t tMs)
{
  CurState = next;

  switch (next)
  {
    case DISP_IDLE:
      ES_Timer_StopTimer(DISPENSE_TIMER);
      break;

    case DISP_ARM_DOWN:
      BucketArmDown();
      ES_Timer_InitTimer(DISPENSE_TIMER, tMs);
      break;

    case DISP_GATE_OPEN:
      if (NextIsFull) GateOpenFull();
      else           GateOpenHalf();
      ES_Timer_InitTimer(DISPENSE_TIMER, tMs);
      break;

    case DISP_WAIT:
      ES_Timer_InitTimer(DISPENSE_TIMER, tMs);
      break;

    case DISP_GATE_CLOSE:
      GateClose();
      ES_Timer_InitTimer(DISPENSE_TIMER, tMs);
      break;

    case DISP_ARM_UP:
      BucketArmUp();
      ES_Timer_InitTimer(DISPENSE_TIMER, tMs);
      break;

    case DISP_DONE:
    default:
      ES_Timer_StopTimer(DISPENSE_TIMER);
      break;
  }
}

/*=========================== SERVO ACTIONS ============================*/
static void BucketArmDown(void) { ServoSet_OC5_us(US_BUCKET_ARM_DOWN); }
static void BucketArmUp(void)   { ServoSet_OC5_us(US_BUCKET_ARM_UP); }

static void GateClose(void)     { ServoSet_OC3_us(US_GATE_CLOSED); }
static void GateOpenHalf(void)  { ServoSet_OC3_us(US_GATE_HALF_OPEN); }
static void GateOpenFull(void)  { ServoSet_OC3_us(US_GATE_FULL_OPEN); }

/*=========================== SERVO PWM INIT ============================*/
static void InitServoPWM(void)
{
  T2CON = 0;
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS;
  TMR2 = 0;
  PR2 = (uint16_t)SERVO_PR2_VALUE;

  OC3CON = 0; OC3R = 0; OC3RS = 0; OC3CONbits.OCTSEL = 0; OC3CONbits.OCM = 0b110; OC3CONbits.ON = 1;
  OC5CON = 0; OC5R = 0; OC5RS = 0; OC5CONbits.OCTSEL = 0; OC5CONbits.OCM = 0b110; OC5CONbits.ON = 1;

  T2CONbits.ON = 1;

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void ServoSet_OC3_us(uint16_t us) { OC3RS = UsToOCrs(us); }
static void ServoSet_OC5_us(uint16_t us) { OC5RS = UsToOCrs(us); }