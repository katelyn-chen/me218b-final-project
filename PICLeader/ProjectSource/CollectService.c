/****************************************************************************
  Module
    CollectService.c  (ARM = 120deg max, GRAB = full 270deg)

  Summary
    - 6-ball collection routine at the dispenser
    - Uses TWO different servos:
        * OC4 (RA4) = ARM servo (35kg)   -> LIMITED motion (about 120 degrees)
        * OC1 (RB4) = GRABBER rotation   -> FULL open/close (about 270 degrees)
    - On START: force arm to BUCKET/READY pose first.
    - Per ball:
        1) Arm DOWN to dispenser
        2) Grabber CLOSE
        3) Arm UP to bucket
        4) Grabber OPEN (deposit)
        5) Repeat until 6 balls
    - Posts ES_COLLECT_DONE after all 6 balls are deposited (handshake for dispense)

  Notes
    - PWM timebase is Timer2 at 50 Hz (shared)
****************************************************************************/

#include "CollectService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"
#include "SPILeaderService.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ               20000000u
#define SERVO_HZ               50u

#define T2_PRESCALE_BITS       0b011u

#ifndef COLLECT_TIMER
#define COLLECT_TIMER          1u
#endif

#define BALL_TARGET_COUNT      6u

/*====================== SERVO VALUES (TUNE ON ROBOT) ======================
  Values are written directly to OCxRS (Timer2 tick units).
============================================================================*/

/* Grabber (OC1 -> RB4): open/close */
// changes travel distance
#define GRAB_OPEN_TICKS        4700u // tested!!! DONT CHANGE VALUES
#define GRAB_CLOSE_TICKS       1000u

/* Arm (OC4 -> RA4): ready/down */
#define ARM_UP_TICKS           1540u // tested!!! DONT CHANGE VALUES
#define ARM_DOWN_TICKS         3400u
// 1540, 3300 is 90 degree 

#define BUCKET_ARM_DISPENSE_TICKS    3000u 
#define BUCKET_ARM_COLLECT_TICKS     5000u

/* Safety clamp for arm motion */
#define ARM_MIN_TICKS          1400u
#define ARM_MAX_TICKS          3400u

/*============================== TIMING ==============================*/
#define T_READY_MS             900u
#define T_ARM_MOVE_MS          1900u
#define T_GRAB_MOVE_MS         1950u
#define T_RELEASE_MS           700u

/* optional nudges */
#define T_NUDGE_MS             550u

#ifndef CMD_NUDGE_BACK
#define CMD_NUDGE_BACK         0x22u
#endif
#ifndef CMD_NUDGE_FWD
#define CMD_NUDGE_FWD          0x23u
#endif

/*============================== STATE ==============================*/
typedef enum {
  COLLECT_IDLE = 0,
  COLLECT_GO_READY,

  COLLECT_ARM_DOWN,
  COLLECT_GRAB_CLOSE,
  COLLECT_ARM_UP,
  COLLECT_GRAB_OPEN,

  COLLECT_CHECK_COUNT,
  COLLECT_DONE
} CollectState_t;

static uint8_t MyPriority;
static CollectState_t CurState = COLLECT_IDLE;
static uint8_t BallCount = 0u;
static CollectStatus_t whichCollect;
static bool post = true;
/*=========================== SERVO HW ============================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);

/* low-level setters */
static inline uint16_t ClampArm(uint16_t v)
{
  if (v < ARM_MIN_TICKS) return ARM_MIN_TICKS;
  if (v > ARM_MAX_TICKS) return ARM_MAX_TICKS;
  return v;
}

static void SetGrabber(uint16_t ticks) { OC2RS = ticks; }
static void SetArm(uint16_t ticks)     { OC1RS = ClampArm(ticks); }
static void SetBucketArm(uint16_t ticks)     { OC4RS = ticks; }
/*=========================== HELPERS ============================*/
static void TransitionTo(CollectState_t next, uint16_t ms);
static void PostCollectDoneHandshake(void);
static void RequestNudge(uint8_t cmdByte);

/*=========================== PUBLIC API ============================*/
bool InitCollectService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = COLLECT_IDLE;
  BallCount = 0u;

  if (!ServoInitDone) InitServoPWM();

  /* Start safe: arm up/ready, grabber open */
  SetArm(ARM_UP_TICKS);
  SetGrabber(GRAB_OPEN_TICKS);

  ES_Timer_StopTimer(COLLECT_TIMER);

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

      if ((ThisEvent.EventType == ES_START_BUTTON) ||
          (ThisEvent.EventType == ES_COLLECT_START))
      {
        DB_printf("CollectService start\r\n");
        BallCount = 0u;

        whichCollect = FIRST_COLLECT;
        if (ThisEvent.EventType == ES_COLLECT_START) {
          whichCollect = ThisEvent.EventParam;
        }

        TransitionTo(COLLECT_GO_READY, 0);
      }
      
      break;

    case COLLECT_GO_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_DOWN, 0);
      }
      break;

    case COLLECT_ARM_DOWN:
    {
        if (BallCount == 0 && post == true) {
            ES_Event_t BucketLowerEvent;
            BucketLowerEvent.EventType = ES_WAIT_BALL;
            PostDispenseService(BucketLowerEvent);
            DB_printf("posted lower bucket \n");
            post = false;
        }
      if (ThisEvent.EventType == ES_BUCKET_READY)
        {
          SetBucketArm(BUCKET_ARM_COLLECT_TICKS);
          DB_printf("bucket ready cmd received\r\n");
        }
        if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRAB_CLOSE, 0);
      }
      break;
    }
    case COLLECT_GRAB_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_UP, 0);
      }
      break;

    case COLLECT_ARM_UP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRAB_OPEN, 0);
      }
      break;

    case COLLECT_GRAB_OPEN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_CHECK_COUNT, 0);
      }
      break;

    case COLLECT_CHECK_COUNT:
      BallCount++;
      DB_printf("Collect ball count = %u\r\n", (unsigned)BallCount);

      if (BallCount < BALL_TARGET_COUNT) {
        TransitionTo(COLLECT_ARM_DOWN, 0);
      } else {
        TransitionTo(COLLECT_DONE, 0);
      }
      break;

    case COLLECT_DONE:
    {
      DB_printf("Collect done\r\n");

      /* safe final pose */
      SetGrabber(GRAB_OPEN_TICKS);
      SetArm(ARM_UP_TICKS);

      PostCollectDoneHandshake();

      /* optional: keep SPI leader done messages */
      ES_Event_t doneEvt;
      doneEvt.EventType = ES_CMD_REQ;

      switch (whichCollect) {
        case FIRST_COLLECT:  doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
        case SECOND_COLLECT: doneEvt.EventParam = CMD_SECOND_COLLECT_DONE; break;
        case OTHER_COLLECT:  doneEvt.EventParam = CMD_OTHER_COLLECT_DONE;  break;
        default:             doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
      }
      PostSPILeaderService(doneEvt);

      TransitionTo(COLLECT_IDLE, 0);
      break;
    }

    default:
      TransitionTo(COLLECT_IDLE, 0);
      break;
  }

  return ReturnEvent;
}

/*=========================== TRANSITIONS ============================*/
static void TransitionTo(CollectState_t next, uint16_t ms)
{
  CurState = next;

  switch (next)
  {
    case COLLECT_IDLE:
      ES_Timer_StopTimer(COLLECT_TIMER);
      break;

    case COLLECT_GO_READY:
      SetArm(ARM_UP_TICKS);
      SetGrabber(GRAB_OPEN_TICKS);
      ES_Timer_InitTimer(COLLECT_TIMER, (ms == 0) ? T_READY_MS : ms);
      break;

    case COLLECT_ARM_DOWN:
      SetArm(ARM_DOWN_TICKS);
      ES_Timer_InitTimer(COLLECT_TIMER, (ms == 0) ? T_ARM_MOVE_MS : ms);
      break;

    case COLLECT_GRAB_CLOSE:
      SetGrabber(GRAB_CLOSE_TICKS);
      ES_Timer_InitTimer(COLLECT_TIMER, (ms == 0) ? T_GRAB_MOVE_MS : ms);
      break;

    case COLLECT_ARM_UP:
      SetArm(ARM_UP_TICKS);
      ES_Timer_InitTimer(COLLECT_TIMER, (ms == 0) ? T_ARM_MOVE_MS : ms);
      break;

    case COLLECT_GRAB_OPEN:
      SetGrabber(GRAB_OPEN_TICKS);
      ES_Timer_InitTimer(COLLECT_TIMER, (ms == 0) ? T_RELEASE_MS : ms);
      break;

    case COLLECT_CHECK_COUNT:
    case COLLECT_DONE:
    default:
      ES_Timer_StopTimer(COLLECT_TIMER);
      ES_Event_t e = { ES_TIMEOUT, COLLECT_TIMER };
      PostCollectService(e);
      break;
  }
}

/*=========================== HANDSHAKE ============================*/
static void PostCollectDoneHandshake(void)
{
  ES_Event_t e = { ES_COLLECT_DONE, 0 };
  ES_PostAll(e);
}

/*=========================== SPI NUDGE ============================*/
static void RequestNudge(uint8_t cmdByte)
{
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*=========================== SERVO PWM INIT ============================*/
static void InitServoPWM(void)
{
  ANSELAbits.ANSA1 = 0;

  TRISBbits.TRISB4 = 0;
  TRISAbits.TRISA4 = 0;
  TRISAbits.TRISA1 = 0;

  /* Timer2 init only if OFF (shared timebase) */
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCKPS = T2_PRESCALE_BITS;
    TMR2 = 0;
    PR2 = 49999u; /* 20ms at PBCLK=20MHz prescale 1:8 */
    T2CONbits.ON = 1;
  }

  /* OC1 grabber arm */
  OC1CON = 0;
  OC1CONbits.OCTSEL = 0;
  OC1R  = 0;
  OC1RS = 0;
  OC1CONbits.OCM = 0b110;
  OC1CONbits.ON  = 1;

  /* OC2 rack and pinion*/
  OC2CON = 0;
  OC2CONbits.OCTSEL = 0;
  OC2R  = 0;
  OC2RS = 0;
  OC2CONbits.OCM = 0b110;
  OC2CONbits.ON  = 1;

  /* OC4 bucket arm */
  OC4CON = 0;
  OC4CONbits.OCTSEL = 0;
  OC4R  = 0;
  OC4RS = 0;
  OC4CONbits.OCM = 0b110;
  OC4CONbits.ON  = 1;

  /* PPS mapping */
  RPB4Rbits.RPB4R = 0b0101;  /* OC1 -> RB4 */
  RPA4Rbits.RPA4R = 0b0101;  /* OC4 -> RA4 */
  RPA1Rbits.RPA1R = 0b0101; /* OC2 -> RA1 */

  ServoInitDone = true;

  DB_printf("Collect PWM init: PR2=%u\r\n", (unsigned)PR2);
}