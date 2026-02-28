/****************************************************************************
  Module
    CollectService.c

  Summary
    - 6-ball collection routine at the dispenser
    - Now with SLOW SERVO RAMPING + full 0°->270° sweep for grabber rotation servo
    - Sequence per ball:
        1) Arm down (more down)
        2) Ensure grabber rotation at 0°
        3) Rotate grabber CW to 270° to grab
        4) Arm up to bucket
        5) Rotate grabber CCW back to 0° to release
        6) Optional nudge FWD/BACK (kept)
        7) Repeat until 6 balls

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

/*========================= SERVO SPEED (RAMP) =========================
  Slows servos by stepping pulse width gradually.
  Tune:
    - SERVO_STEP_US smaller => slower/smoother
    - SERVO_STEP_MS bigger  => slower updates
*/
#define SERVO_STEP_US          5u     /* try 3..15 */
#define SERVO_STEP_MS          10u    /* try 10..30 */

/*=========================== SERVO PWM HW ============================*/
/*
  Servo outputs (PIC1 pins):
    OC2 -> RA1  Grab Servo 1 (rack/pinion)         [optional / not used for 0->270 sweep]
    OC1 -> RB4  Grab Servo 2 (grabber rotation)    [THIS does 0->270 sweep]
    OC4 -> RA4  Rotate Servo 1 (lever arm)         [arm up/down]
*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u

/* 50 Hz: 20ms period */
#define SERVO_HZ                    150u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/* ========= PULSE WIDTHS (TUNE THESE ON ROBOT) =========
   IMPORTANT: “270° servo” still expects ~50Hz pulses.
   The 0°/270° endpoints are NOT universal; you must tune.

   Start with safe range 900..2100 and expand only if needed.
*/
#define US_ROT_0_DEG                900u    /* rotation servo: 0° (open)  (tune) */
#define US_ROT_270_DEG              2100u   /* rotation servo: 270° (closed/grab) (tune) */

#define US_RACK_OPEN                1900u   /* OC2 (rack/pinion) if you still use it */
#define US_RACK_CLOSE               1100u

/* Arm (lever arm) — you wanted “more down” */
#define US_ARM_DOWN_MORE            2300u   /* DOWN to ground (tune: 2050..2400) */
#define US_ARM_UP_BUCKET            1100u   /* UP to bucket */
#define US_ARM_TRAVEL               1500u   /* safe mid pose */

static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);
static void ServoSet_OC1_us(uint16_t us);
static void ServoSet_OC2_us(uint16_t us);
static void ServoSet_OC4_us(uint16_t us);

/*============================== TIMING ==============================*/
/* These now mostly represent "how long we wait AFTER commanding" for each phase,
   but actual motion speed is governed by the ramp (SERVO_STEP_US/MS). */
#define T_ARM_SETTLE_MS        150u
#define T_GRAB_SETTLE_MS       150u
#define T_RELEASE_SETTLE_MS    150u
#define T_NUDGE_MS             300u   /* still field-tuned */

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

  /* per-ball sequence */
  COLLECT_ARM_DOWN,
  COLLECT_SET_GRABBER_0,
  COLLECT_GRABBER_TO_270,
  COLLECT_NUDGE_BACK,
  COLLECT_ARM_UP_BUCKET,
  COLLECT_RELEASE_TO_0,
  COLLECT_NUDGE_FWD,
  COLLECT_TRAVEL_POSE,

  COLLECT_CHECK_COUNT,
  COLLECT_DONE
} CollectState_t;

static uint8_t MyPriority;
static CollectState_t CurState = COLLECT_IDLE;
static uint8_t BallCount = 0u;
static CollectStatus_t whichCollect;

/*====================== SERVO RAMP ENGINE =====================*/
static volatile uint16_t oc1_cur = US_ROT_0_DEG;
static volatile uint16_t oc2_cur = US_RACK_OPEN;
static volatile uint16_t oc4_cur = US_ARM_TRAVEL;

static volatile uint16_t oc1_tgt = US_ROT_0_DEG;
static volatile uint16_t oc2_tgt = US_RACK_OPEN;
static volatile uint16_t oc4_tgt = US_ARM_TRAVEL;

static uint8_t ServoRampTimerNum = 15u; /* default; MUST map TIMER15_RESP_FUNC to PostCollectService */
static bool rampActive = false;

static uint16_t StepToward(uint16_t cur, uint16_t tgt)
{
  if (cur == tgt) return cur;
  if (cur < tgt) {
    uint16_t next = cur + SERVO_STEP_US;
    return (next > tgt) ? tgt : next;
  } else {
    uint16_t next = (cur > SERVO_STEP_US) ? (cur - SERVO_STEP_US) : 0u;
    return (next < tgt) ? tgt : next;
  }
}

static void ServoWriteAll(void)
{
  ServoSet_OC1_us(oc1_cur);
  ServoSet_OC2_us(oc2_cur);
  ServoSet_OC4_us(oc4_cur);
}

/* Set targets; ramp engine will move gradually */
static void ServoRampTo(uint16_t oc1_us, uint16_t oc2_us, uint16_t oc4_us)
{
  oc1_tgt = oc1_us;
  oc2_tgt = oc2_us;
  oc4_tgt = oc4_us;

  rampActive = true;
  ES_Timer_InitTimer(ServoRampTimerNum, SERVO_STEP_MS);
}

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void TransitionTo(CollectState_t next, uint16_t tMs);
static void RequestNudge(uint8_t cmdByte);

/* high-level actions */
static void ArmDownToGround(void);
static void ArmUpToBucket(void);
static void ArmTravelPose(void);

static void GrabberRotTo0(void);
static void GrabberRotTo270(void);

/* Optional rack/pinion if you still want it (kept but not required) */
static void RackOpen(void);
static void RackClose(void);

/*=========================== PUBLIC API =============================*/
bool InitCollectService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = COLLECT_IDLE;
  BallCount = 0u;

  if (!ServoInitDone) InitServoPWM();

  /* Initialize commanded positions */
  oc1_cur = US_ROT_0_DEG;   oc1_tgt = US_ROT_0_DEG;
  oc2_cur = US_RACK_OPEN;   oc2_tgt = US_RACK_OPEN;
  oc4_cur = US_ARM_TRAVEL;  oc4_tgt = US_ARM_TRAVEL;
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

  /* ================= SERVO RAMP TIMER HANDLER =================
     NOTE: This requires TIMER15_RESP_FUNC = PostCollectService in ES_Configure.h.
  */
  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ServoRampTimerNum))
  {
    if (rampActive)
    {
      oc1_cur = StepToward(oc1_cur, oc1_tgt);
      oc2_cur = StepToward(oc2_cur, oc2_tgt);
      oc4_cur = StepToward(oc4_cur, oc4_tgt);
      ServoWriteAll();

      if ((oc1_cur == oc1_tgt) && (oc2_cur == oc2_tgt) && (oc4_cur == oc4_tgt)) {
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
        whichCollect = FIRST_COLLECT; /* safe default */
        TransitionTo(COLLECT_ARM_DOWN, 0u);
      }

      if (ThisEvent.EventType == ES_COLLECT_START)
      {
        DB_printf("CollectService: start (event)\r\n");
        whichCollect = ThisEvent.EventParam;
        BallCount = 0u;
        TransitionTo(COLLECT_ARM_DOWN, 0u);
      }
      break;

    case COLLECT_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_SET_GRABBER_0, 0u);
      }
      break;

    case COLLECT_SET_GRABBER_0:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_GRABBER_TO_270, 0u);
      }
      break;

    case COLLECT_GRABBER_TO_270:
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
        TransitionTo(COLLECT_RELEASE_TO_0, 0u);
      }
      break;

    case COLLECT_RELEASE_TO_0:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_NUDGE_FWD, 0u);
      }
      break;

    case COLLECT_NUDGE_FWD:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
      {
        TransitionTo(COLLECT_TRAVEL_POSE, 0u);
      }
      break;

    case COLLECT_TRAVEL_POSE:
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
      /* go safe */
      GrabberRotTo0();
      RackOpen();
      ArmTravelPose();

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

    case COLLECT_ARM_DOWN:
      /* Arm goes DOWN more (to ground) */
      ArmDownToGround();
      /* Ensure rack open (optional) */
      RackOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_ARM_SETTLE_MS : tMs);
      break;

    case COLLECT_SET_GRABBER_0:
      GrabberRotTo0();
      ES_Timer_InitTimer(COLLECT_TIMER, T_GRAB_SETTLE_MS);
      break;

    case COLLECT_GRABBER_TO_270:
      /* Sweep CW to 270° to grab */
      GrabberRotTo270();
      /* optional: tighten rack too if you want */
      RackClose();
      ES_Timer_InitTimer(COLLECT_TIMER, T_GRAB_SETTLE_MS);
      break;

    case COLLECT_NUDGE_BACK:
      RequestNudge(CMD_NUDGE_BACK);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
      break;

    case COLLECT_ARM_UP_BUCKET:
      ArmUpToBucket();
      ES_Timer_InitTimer(COLLECT_TIMER, T_ARM_SETTLE_MS);
      break;

    case COLLECT_RELEASE_TO_0:
      /* Release by rotating back CCW to 0° */
      GrabberRotTo0();
      RackOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, T_RELEASE_SETTLE_MS);
      break;

    case COLLECT_NUDGE_FWD:
      RequestNudge(CMD_NUDGE_FWD);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
      break;

    case COLLECT_TRAVEL_POSE:
      ArmTravelPose();
      ES_Timer_InitTimer(COLLECT_TIMER, T_ARM_SETTLE_MS);
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
      SPILeaderService is the owner of SPI on PIC1 and is polling.
      Request nudges by posting an event so it can queue+send safely.
  */
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;   /* <-- FIX: SPILeaderService listens for ES_CMD_REQ */
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*=========================== HIGH-LEVEL SERVO ACTIONS ============================*/
static void ArmDownToGround(void)
{
  ServoRampTo(oc1_tgt, oc2_tgt, US_ARM_DOWN_MORE);
}

static void ArmUpToBucket(void)
{
  ServoRampTo(oc1_tgt, oc2_tgt, US_ARM_UP_BUCKET);
}

static void ArmTravelPose(void)
{
  ServoRampTo(oc1_tgt, oc2_tgt, US_ARM_TRAVEL);
}

static void GrabberRotTo0(void)
{
  ServoRampTo(US_ROT_0_DEG, oc2_tgt, oc4_tgt);
}

static void GrabberRotTo270(void)
{
  ServoRampTo(US_ROT_270_DEG, oc2_tgt, oc4_tgt);
}

static void RackOpen(void)
{
  /* if OC2 rack/pinion servo is physically used */
  ServoRampTo(oc1_tgt, US_RACK_OPEN, oc4_tgt);
}

static void RackClose(void)
{
  ServoRampTo(oc1_tgt, US_RACK_CLOSE, oc4_tgt);
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
    T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS;   /* 1:8 */
    TMR2 = 0;
    PR2 = (uint16_t)SERVO_PR2_VALUE;               /* ~49999 for 20ms */

    T2CONbits.ON = 1;
  }

  /*
    Ensure OC modules are in PWM mode on Timer2.
  */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC2CON = 0; OC2R = 0; OC2RS = 0; OC2CONbits.OCTSEL = 0; OC2CONbits.OCM = 0b110; OC2CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;

  /* ================= PPS + PIN CONFIG =================
     OC modules do NOT automatically appear on pins.
     You MUST map them via PPS or the servo will not move.

     OC2 -> RA1 (rack/pinion)
     OC4 -> RA4 (arm)
     OC1 -> RB4 (rotation)
  */
#ifdef _ANSELA_ANSA1_MASK
  ANSELACLR = _ANSELA_ANSA1_MASK;   /* RA1 digital (AN1) */
#endif
#ifdef _ANSELB_ANSB4_MASK
  ANSELBCLR = _ANSELB_ANSB4_MASK;   /* RB4 digital (if analog-capable) */
#endif

  TRISAbits.TRISA1 = 0;
  TRISAbits.TRISA4 = 0;
  TRISBbits.TRISB4 = 0;

#ifndef PPS_FN_PWM
#define PPS_FN_PWM 0b0101
#endif
  /* PPS output select */
  RPA1Rbits.RPA1R = PPS_FN_PWM;  /* OC2 -> RA1 */
  RPA4Rbits.RPA4R = PPS_FN_PWM;  /* OC4 -> RA4 */
  RPB4Rbits.RPB4R = PPS_FN_PWM;  /* OC1 -> RB4 */

  ServoInitDone = true;
  DB_printf("Collect PWM init ok (Timer2 50Hz). Servo ramp timer uses ES timer %u.\r\n",
            (unsigned)ServoRampTimerNum);
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