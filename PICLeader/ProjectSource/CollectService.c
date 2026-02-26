/****************************************************************************
  Module
    CollectService.c

  Summary
    - Runs the full 6-ball collection routine at the dispenser
    - So logic is: Arm down -> close gripper -> nudge back -> arm up/drop -> nudge forward
    - Repeats until 6 balls are collected
    - All nudges are requested by sending SPI command bytes to the follower

  Notes
    - Servo PWM is generated on this PIC (Leader)
    - Motor motion happens on follower PIC; this service requests nudges via SPI
****************************************************************************/

#include "CollectService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"
#include "PIC32_SPI_HAL.h"
#include "SPILeaderService.h"   /* command bytes live in SPI header now */

/*============================== CONFIG ==============================*/
#define PBCLK_HZ               20000000u

#ifndef COLLECT_TIMER
#define COLLECT_TIMER          1u
#endif

/* collect target */
#define BALL_TARGET_COUNT      6u

/* timing (ms) */
#define T_ARM_DOWN_MS          450u
#define T_GRIP_CLOSE_MS        350u
#define T_NUDGE_MS             300u   /* we need to be test on field and tune */
#define T_ARM_UP_DROP_MS       450u
#define T_GRIP_OPEN_MS         250u
#define T_ARM_TRAVEL_MS        350u

/*=========================== SERVO PWM HW ============================*/
/*
  Servo outputs (PIC1 pins):
    OC2 -> RA1  Grab Servo 1 (rack/pinion)
    OC1 -> RB4  Grab Servo 2 (grabber rotation)
    OC4 -> RA4  Rotate Servo 1 (lever arm)
    OC5 -> RB6  Lower Arm Servo 1
    OC3 -> RB10 Rotate Servo 2 (bucket)

  This service uses:
    - Grab servos (OC2/OC1)
    - Lever arm (OC4)
*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u

/* 50 Hz: 20ms period */
#define SERVO_HZ                    50u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/* pulse widths in microseconds (tune these on the robot) */
#define US_GRIP_OPEN_1              1900u
#define US_GRIP_CLOSE_1             1100u

#define US_GRIP_OPEN_2              1800u
#define US_GRIP_CLOSE_2             1200u

#define US_ARM_PICK                 2000u   /* down to dispenser */
#define US_ARM_DROP                 1100u   /* up to bucket */
#define US_ARM_TRAVEL               1500u   /* safe mid pose */

static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);
static void ServoSet_OC1_us(uint16_t us);
static void ServoSet_OC2_us(uint16_t us);
static void ServoSet_OC4_us(uint16_t us);

/*============================== STATE ==============================*/
typedef enum {
  COLLECT_IDLE = 0,
  COLLECT_ARM_DOWN,
  COLLECT_GRIP_CLOSE,
  COLLECT_NUDGE_BACK,
  COLLECT_ARM_UP_DROP,
  COLLECT_GRIP_OPEN,
  COLLECT_NUDGE_FWD,
  COLLECT_ARM_TRAVEL,
  COLLECT_CHECK_COUNT,
  COLLECT_DONE
} CollectState_t;

static uint8_t MyPriority;
static CollectState_t CurState = COLLECT_IDLE;
static uint8_t BallCount = 0u;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void TransitionTo(CollectState_t next, uint16_t tMs);
static void RequestNudge(uint8_t cmdByte);

static void GrabOpen(void);
static void GrabClose(void);
static void ArmPickPose(void);
static void ArmDropPose(void);
static void ArmTravelPose(void);

/*=========================== PUBLIC API =============================*/
bool InitCollectService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = COLLECT_IDLE;
  BallCount = 0u;

  if (!ServoInitDone) InitServoPWM();

  GrabOpen();
  ArmTravelPose();

  ES_Timer_StopTimer(COLLECT_TIMER);

  DB_printf("CollectService: init done\r\n");

  ES_Event_t e = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, e);
}

bool PostCollectService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunCollectService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (CurState)
  {
    case COLLECT_IDLE:
      /*
        IMPORTANT FOR DEBUG:
          Start Collect on the same capacitive button your framework posts as ES_START_BUTTON.
          This lets you test CollectService without running the whole game.
      */
      if (ThisEvent.EventType == ES_START_BUTTON)
      {
        DB_printf("CollectService: start (ES_START_BUTTON)\r\n");
        BallCount = 0u;
        TransitionTo(COLLECT_ARM_DOWN, T_ARM_DOWN_MS);
      }

      if (ThisEvent.EventType == ES_COLLECT_START)
      {
        DB_printf("CollectService: start (ES_COLLECT_START)\r\n");
        BallCount = 0u;
        TransitionTo(COLLECT_ARM_DOWN, T_ARM_DOWN_MS);
      }
      break;

    case COLLECT_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRIP_CLOSE, T_GRIP_CLOSE_MS);
      }
      break;

    case COLLECT_GRIP_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_NUDGE_BACK, T_NUDGE_MS);
      }
      break;

    case COLLECT_NUDGE_BACK:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_UP_DROP, T_ARM_UP_DROP_MS);
      }
      break;

    case COLLECT_ARM_UP_DROP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRIP_OPEN, T_GRIP_OPEN_MS);
      }
      break;

    case COLLECT_GRIP_OPEN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_NUDGE_FWD, T_NUDGE_MS);
      }
      break;

    case COLLECT_NUDGE_FWD:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_TRAVEL, T_ARM_TRAVEL_MS);
      }
      break;

    case COLLECT_ARM_TRAVEL:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_CHECK_COUNT, 0u);
      }
      break;

    case COLLECT_CHECK_COUNT:
      BallCount++;
      DB_printf("CollectService: ball=%u\r\n", (unsigned)BallCount);

      if (BallCount < BALL_TARGET_COUNT)
      {
        TransitionTo(COLLECT_ARM_DOWN, T_ARM_DOWN_MS);
      }
      else
      {
        TransitionTo(COLLECT_DONE, 0u);
      }
      break;

    case COLLECT_DONE:
    {
      DB_printf("CollectService: done\r\n");
      GrabOpen();
      ArmTravelPose();

      ES_Event_t doneEvt = { ES_COLLECT_DONE, 0 };
      ES_PostAll(doneEvt);

      TransitionTo(COLLECT_IDLE, 0u);
      break;
    }

    default:
      TransitionTo(COLLECT_IDLE, 0u);
      break;
  }

  return ReturnEvent;
}

/*=========================== TRANSITIONS ============================*/
static void TransitionTo(CollectState_t next, uint16_t tMs)
{
  CurState = next;

  switch (next)
  {
    case COLLECT_IDLE:
      ES_Timer_StopTimer(COLLECT_TIMER);
      break;

    case COLLECT_ARM_DOWN:
      ArmPickPose();
      GrabOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_GRIP_CLOSE:
      GrabClose();
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_NUDGE_BACK:
      RequestNudge(CMD_NUDGE_BACK);
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_ARM_UP_DROP:
      ArmDropPose();
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_GRIP_OPEN:
      GrabOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_NUDGE_FWD:
      RequestNudge(CMD_NUDGE_FWD);
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_ARM_TRAVEL:
      ArmTravelPose();
      ES_Timer_InitTimer(COLLECT_TIMER, tMs);
      break;

    case COLLECT_CHECK_COUNT:
    case COLLECT_DONE:
    default:
      ES_Timer_StopTimer(COLLECT_TIMER);
      break;
  }
}

/*=========================== SPI NUDGE ============================*/
static void RequestNudge(uint8_t cmdByte)
{
  /*
    IMPORTANT:
      Don't call SPIOperate_SPI1_Send8Wait() directly here.
      SPILeaderService is already the owner of SPI on PIC1 and is polling.
      We request nudges by posting ES_COMMAND_RECEIVED so it can queue+send safely.
  */
  ES_Event_t CmdEvent;
  CmdEvent.EventType = ES_COMMAND_RECEIVED;
  CmdEvent.EventParam = cmdByte;
  ES_PostAll(CmdEvent);
}

/*=========================== SERVO POSES ============================*/
static void GrabOpen(void)
{
  ServoSet_OC2_us(US_GRIP_OPEN_1);
  ServoSet_OC1_us(US_GRIP_OPEN_2);
}

static void GrabClose(void)
{
  ServoSet_OC2_us(US_GRIP_CLOSE_1);
  ServoSet_OC1_us(US_GRIP_CLOSE_2);
}

static void ArmPickPose(void)
{
  ServoSet_OC4_us(US_ARM_PICK);
}

static void ArmDropPose(void)
{
  ServoSet_OC4_us(US_ARM_DROP);
}

static void ArmTravelPose(void)
{
  ServoSet_OC4_us(US_ARM_TRAVEL);
}

/*=========================== SERVO PWM INIT ============================*/
static void InitServoPWM(void)
{
  /*
    IMPORTANT:
      CollectService and DispenseService both use Timer2 as the 50Hz servo timebase.
      We must NOT reinitialize Timer2 if it is already running, otherwise we can stomp
      OC settings used by the other service.
  */

  if (T2CONbits.ON == 0u)
  {
    /* Timer2 as 50Hz timebase for all OC servo outputs */
    T2CON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS;
    TMR2 = 0;
    PR2 = (uint16_t)SERVO_PR2_VALUE;

    T2CONbits.ON = 1;
  }

  /*
    Ensure ALL OC modules used by any servo service are in PWM mode on Timer2.
    (OC1/OC2/OC4 used here; OC3/OC5 used by DispenseService)
  */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC2CON = 0; OC2R = 0; OC2RS = 0; OC2CONbits.OCTSEL = 0; OC2CONbits.OCM = 0b110; OC2CONbits.ON = 1;
  OC3CON = 0; OC3R = 0; OC3RS = 0; OC3CONbits.OCTSEL = 0; OC3CONbits.OCM = 0b110; OC3CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;
  OC5CON = 0; OC5R = 0; OC5RS = 0; OC5CONbits.OCTSEL = 0; OC5CONbits.OCM = 0b110; OC5CONbits.ON = 1;

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* tickRate = PBCLK/8 = 2.5 MHz -> 1 us = 2.5 ticks */
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void ServoSet_OC1_us(uint16_t us) { OC1RS = UsToOCrs(us); }
static void ServoSet_OC2_us(uint16_t us) { OC2RS = UsToOCrs(us); }
static void ServoSet_OC4_us(uint16_t us) { OC4RS = UsToOCrs(us); }