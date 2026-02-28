/****************************************************************************
  Module
    CollectService.c  (ARM = 120deg max, GRAB = full 270deg, correct start logic)

  Summary
    - 6-ball collection routine at the dispenser
    - Uses TWO different servos:
        * OC4 (RA4) = ARM servo (35kg)   -> LIMITED to 120 degrees
        * OC1 (RB4) = GRABBER rotation   -> FULL 270 degrees
    - On START: force arm to BUCKET/READY pose first.
    - Per ball:
        1) Arm DOWN to dispenser (<=120deg)
        2) Grabber CLOSE (0->270)
        3) Arm UP to bucket
        4) Grabber OPEN (270->0)
        5) Repeat until 6 balls
    - Includes slow ramping (separate step sizes for ARM and GRAB).

  Notes
    - PWM timebase is Timer2 at 50 Hz.
    - Ramp uses an ES timer (ServoRampTimerNum) that must route to PostCollectService.
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
#include "SPILeaderService.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ               20000000u

#ifndef COLLECT_TIMER
#define COLLECT_TIMER          1u
#endif

#define BALL_TARGET_COUNT      6u

/*========================= SERVO RAMP (PER SERVO) =========================*/
#define SERVO_STEP_MS              10u
#define ARM_STEP_US                4u   /* heavier arm -> slower */
#define GRAB_STEP_US               6u   /* grabber can be faster */

/*=========================== SERVO PWM HW ============================*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u

#define SERVO_HZ                    50u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/*==================== SERVO PWM RANGES / LIMITS =====================*/
/* Grabber full 270deg range */
#define GRAB_MIN_US                 500u
#define GRAB_MAX_US                 2500u
#define GRAB_NEUTRAL_US             1500u
#define GRAB_OPEN_US                500u     /* 0 deg (OPEN) */
#define GRAB_CLOSE_US               2500u    /* 270 deg (CLOSE) */

/*
  ARM MUST ONLY MOVE 120 degrees.

  Linear assumption: (500..2500) = 2000us span for 270deg.
  120deg span = 2000*(120/270) ~= 889us
  -> ARM_120_DEG_US ~ 500 + 889 = 1389us
*/
#define ARM_0_DEG_US                500u
#define ARM_120_DEG_US              1389u
#define ARM_NEUTRAL_US              ((ARM_0_DEG_US + ARM_120_DEG_US) / 2u)

/* Arm poses (bounded) */
#define ARM_BUCKET_READY_US         ARM_0_DEG_US      /* "ready at bucket" */
#define ARM_DOWN_DISPENSER_US       ARM_120_DEG_US    /* down at dispenser */
#define ARM_TRAVEL_US               ARM_NEUTRAL_US    /* safe mid (optional) */

/*============================== TIMING ==============================*/
/* IMPORTANT: these are "wait after command" timers.
   If your ramp is slow, increase these. */
#define T_READY_SETTLE_MS      250u
#define T_ARM_MOVE_MS          250u
#define T_GRAB_MOVE_MS         250u
#define T_RELEASE_MOVE_MS      250u
#define T_NUDGE_MS             300u

/* SPI bytes for follower motion nudges */
#ifndef CMD_NUDGE_BACK
#define CMD_NUDGE_BACK         0x22u
#endif
#ifndef CMD_NUDGE_FWD
#define CMD_NUDGE_FWD          0x23u
#endif

/*============================== STATE ==============================*/
typedef enum {
  COLLECT_IDLE = 0,

  /* startup */
  COLLECT_GO_READY,        /* force arm to bucket-ready first */

  /* per-ball cycle */
  COLLECT_ARM_DOWN,
  COLLECT_GRAB_CLOSE,
  COLLECT_NUDGE_BACK,
  COLLECT_ARM_UP_BUCKET,
  COLLECT_GRAB_OPEN,
  COLLECT_NUDGE_FWD,

  COLLECT_CHECK_COUNT,
  COLLECT_DONE
} CollectState_t;

static uint8_t MyPriority;
static CollectState_t CurState = COLLECT_IDLE;
static uint8_t BallCount = 0u;
static CollectStatus_t whichCollect;

/*=========================== SERVO INTERNALS ============================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);
static uint16_t ClampGrabUs(uint16_t us);
static uint16_t ClampArmUs(uint16_t us);
static void ServoSet_OC1_us(uint16_t us); /* grabber */
static void ServoSet_OC4_us(uint16_t us); /* arm */

/*====================== SERVO RAMP ENGINE (SEPARATED) =====================*/
static volatile uint16_t grab_cur_us = GRAB_NEUTRAL_US;
static volatile uint16_t grab_tgt_us = GRAB_NEUTRAL_US;

static volatile uint16_t arm_cur_us  = ARM_NEUTRAL_US;
static volatile uint16_t arm_tgt_us  = ARM_NEUTRAL_US;

static uint8_t ServoRampTimerNum = 15u; /* MUST map TIMER15_RESP_FUNC to PostCollectService */
static bool rampActive = false;

static uint16_t StepToward(uint16_t cur, uint16_t tgt, uint16_t stepUs)
{
  if (cur == tgt) return cur;

  if (cur < tgt) {
    uint16_t next = (uint16_t)(cur + stepUs);
    return (next > tgt) ? tgt : next;
  } else {
    uint16_t next = (cur > stepUs) ? (uint16_t)(cur - stepUs) : 0u;
    return (next < tgt) ? tgt : next;
  }
}

static void ServoWriteAll(void)
{
  ServoSet_OC1_us(grab_cur_us);
  ServoSet_OC4_us(arm_cur_us);
}

static void ServoRampTo(uint16_t grab_us, uint16_t arm_us)
{
  grab_tgt_us = ClampGrabUs(grab_us);
  arm_tgt_us  = ClampArmUs(arm_us);

  rampActive = true;
  ES_Timer_InitTimer(ServoRampTimerNum, SERVO_STEP_MS);
}

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void TransitionTo(CollectState_t next, uint16_t tMs);
static void RequestNudge(uint8_t cmdByte);

/* high-level actions */
static void ArmToReady(void);
static void ArmDownToDispenser(void);
static void ArmUpToBucket(void);

static void GrabberOpen(void);
static void GrabberClose(void);

/*=========================== PUBLIC API =============================*/
bool InitCollectService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = COLLECT_IDLE;
  BallCount = 0u;

  if (!ServoInitDone) InitServoPWM();

  /* Start in READY + OPEN */
  grab_cur_us = ClampGrabUs(GRAB_OPEN_US);
  grab_tgt_us = grab_cur_us;

  arm_cur_us  = ClampArmUs(ARM_BUCKET_READY_US);
  arm_tgt_us  = arm_cur_us;

  ServoWriteAll();

  ES_Timer_StopTimer(COLLECT_TIMER);
  ES_Timer_StopTimer(ServoRampTimerNum);

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

  /* ================= SERVO RAMP TIMER HANDLER ================= */
  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ServoRampTimerNum))
  {
    if (rampActive)
    {
      grab_cur_us = StepToward(grab_cur_us, grab_tgt_us, GRAB_STEP_US);
      arm_cur_us  = StepToward(arm_cur_us,  arm_tgt_us,  ARM_STEP_US);

      ServoWriteAll();

      if ((grab_cur_us == grab_tgt_us) && (arm_cur_us == arm_tgt_us)) {
        rampActive = false;
        ES_Timer_StopTimer(ServoRampTimerNum);
      } else {
        ES_Timer_InitTimer(ServoRampTimerNum, SERVO_STEP_MS);
      }
    }
    return ReturnEvent;
  }

  switch (CurState)
  {
    case COLLECT_IDLE:
      if (ThisEvent.EventType == ES_START_BUTTON)
      {
        DB_printf("CollectService: start (button)\r\n");
        BallCount = 0u;
        whichCollect = FIRST_COLLECT;
        TransitionTo(COLLECT_GO_READY, 0u);
      }

      if (ThisEvent.EventType == ES_COLLECT_START)
      {
        DB_printf("CollectService: start (event)\r\n");
        whichCollect = ThisEvent.EventParam;
        BallCount = 0u;
        TransitionTo(COLLECT_GO_READY, 0u);
      }
      break;

    case COLLECT_GO_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_DOWN, 0u);
      }
      break;

    case COLLECT_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRAB_CLOSE, 0u);
      }
      break;

    case COLLECT_GRAB_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_NUDGE_BACK, 0u);
      }
      break;

    case COLLECT_NUDGE_BACK:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_ARM_UP_BUCKET, 0u);
      }
      break;

    case COLLECT_ARM_UP_BUCKET:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRAB_OPEN, 0u);
      }
      break;

    case COLLECT_GRAB_OPEN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_NUDGE_FWD, 0u);
      }
      break;

    case COLLECT_NUDGE_FWD:
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
        TransitionTo(COLLECT_ARM_DOWN, 0u);
      }
      else
      {
        TransitionTo(COLLECT_DONE, 0u);
      }
      break;

    case COLLECT_DONE:
    {
      DB_printf("CollectService: done\r\n");

      /* Safe final pose: READY + OPEN */
      GrabberOpen();
      ArmToReady();

      ES_Event_t doneEvt;
      doneEvt.EventType = ES_CMD_REQ;

      switch (whichCollect) {
        case FIRST_COLLECT:  doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
        case SECOND_COLLECT: doneEvt.EventParam = CMD_SECOND_COLLECT_DONE; break;
        case OTHER_COLLECT:  doneEvt.EventParam = CMD_OTHER_COLLECT_DONE;  break;
        default:             doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
      }

      PostSPILeaderService(doneEvt);
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

    case COLLECT_GO_READY:
      /* Step 1: force bucket-ready pose + open grabber */
      ArmToReady();
      GrabberOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_READY_SETTLE_MS : tMs);
      break;

    case COLLECT_ARM_DOWN:
      /* Step 2: arm down to dispenser (<=120deg) */
      ArmDownToDispenser();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_ARM_MOVE_MS : tMs);
      break;

    case COLLECT_GRAB_CLOSE:
      /* Step 3: close grabber (0->270) */
      GrabberClose();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_GRAB_MOVE_MS : tMs);
      break;

    case COLLECT_NUDGE_BACK:
      RequestNudge(CMD_NUDGE_BACK);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
      break;

    case COLLECT_ARM_UP_BUCKET:
      /* Step 4: arm back up to bucket */
      ArmUpToBucket();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_ARM_MOVE_MS : tMs);
      break;

    case COLLECT_GRAB_OPEN:
      /* Step 5: open grabber (270->0) */
      GrabberOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_RELEASE_MOVE_MS : tMs);
      break;

    case COLLECT_NUDGE_FWD:
      RequestNudge(CMD_NUDGE_FWD);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
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
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*=========================== HIGH-LEVEL SERVO ACTIONS ============================*/
static void ArmToReady(void)
{
  ServoRampTo(grab_tgt_us, ARM_BUCKET_READY_US);
}

static void ArmDownToDispenser(void)
{
  ServoRampTo(grab_tgt_us, ARM_DOWN_DISPENSER_US);
}

static void ArmUpToBucket(void)
{
  ServoRampTo(grab_tgt_us, ARM_BUCKET_READY_US);
}

static void GrabberOpen(void)
{
  ServoRampTo(GRAB_OPEN_US, arm_tgt_us);
}

static void GrabberClose(void)
{
  ServoRampTo(GRAB_CLOSE_US, arm_tgt_us);
}

/*=========================== SERVO PWM INIT ============================*/
static void InitServoPWM(void)
{
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS; /* 1:8 */
    TMR2 = 0;
    PR2 = (uint16_t)SERVO_PR2_VALUE;
    T2CONbits.ON = 1;
  }

  /* OC1 (grabber) and OC4 (arm) in PWM mode on Timer2 */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;

  /* PPS + PIN CONFIG */
#ifdef _ANSELB_ANSB4_MASK
  ANSELBCLR = _ANSELB_ANSB4_MASK;
#endif

  TRISAbits.TRISA4 = 0;
  TRISBbits.TRISB4 = 0;

#ifndef PPS_FN_PWM
#define PPS_FN_PWM 0b0101
#endif

  RPA4Rbits.RPA4R = PPS_FN_PWM;  /* OC4 -> RA4 */
  RPB4Rbits.RPB4R = PPS_FN_PWM;  /* OC1 -> RB4 */

  ServoInitDone = true;
  DB_printf("Collect PWM init ok (Timer2 %u Hz). Ramp timer uses ES timer %u.\r\n",
            (unsigned)SERVO_HZ, (unsigned)ServoRampTimerNum);
}

/*=========================== SERVO LOW-LEVEL ============================*/
static uint16_t ClampGrabUs(uint16_t us)
{
  if (us < GRAB_MIN_US) return GRAB_MIN_US;
  if (us > GRAB_MAX_US) return GRAB_MAX_US;
  return us;
}

static uint16_t ClampArmUs(uint16_t us)
{
  /* Arm HARD LIMITED to 0..120deg */
  if (us < ARM_0_DEG_US)   return ARM_0_DEG_US;
  if (us > ARM_120_DEG_US) return ARM_120_DEG_US;
  return us;
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* tickRate = PBCLK/8 = 2.5 MHz -> 1 us = 2.5 ticks */
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void ServoSet_OC1_us(uint16_t us) { OC1RS = UsToOCrs(ClampGrabUs(us)); }
static void ServoSet_OC4_us(uint16_t us) { OC4RS = UsToOCrs(ClampArmUs(us)); }