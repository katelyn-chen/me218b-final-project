/****************************************************************************
  CollectService.c (FIXED: OC1 grabber actually outputs PWM + ES_COLLECT_DONE)
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

#ifndef COLLECT_TIMER
#define COLLECT_TIMER          1u
#endif

#define BALL_TARGET_COUNT      6u

/*========================= SERVO RAMP =========================*/
#define SERVO_STEP_MS              10u
#define ARM_STEP_US                4u
#define GRAB_STEP_US               6u

/*=========================== SERVO PWM HW ============================*/
#define SERVO_TIMER_PRESCALE_BITS   0b011u   /* 1:8 */
#define SERVO_TIMER_PRESCALE_VAL    8u
#define SERVO_HZ                    50u
#define SERVO_PR2_VALUE             ((PBCLK_HZ / (SERVO_TIMER_PRESCALE_VAL * SERVO_HZ)) - 1u)

/*==================== SERVO PWM LIMITS =====================*/
#define GRAB_MIN_US                 500u
#define GRAB_MAX_US                 2500u
#define GRAB_OPEN_US                500u
#define GRAB_CLOSE_US               2500u

#define ARM_0_DEG_US                500u
#define ARM_120_DEG_US              1389u
#define ARM_BUCKET_READY_US         ARM_0_DEG_US
#define ARM_DOWN_DISPENSER_US       ARM_120_DEG_US

/*============================== TIMING ==============================*/
#define T_READY_SETTLE_MS      250u
#define T_ARM_MOVE_MS          250u
#define T_GRAB_MOVE_MS         250u
#define T_RELEASE_MOVE_MS      250u
#define T_NUDGE_MS             300u

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

static bool ServoInitDone = false;

/*====================== SERVO LOW-LEVEL =====================*/
static uint16_t ClampGrabUs(uint16_t us)
{
  if (us < GRAB_MIN_US) return GRAB_MIN_US;
  if (us > GRAB_MAX_US) return GRAB_MAX_US;
  return us;
}

static uint16_t ClampArmUs(uint16_t us)
{
  if (us < ARM_0_DEG_US) return ARM_0_DEG_US;
  if (us > ARM_120_DEG_US) return ARM_120_DEG_US;
  return us;
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* PBCLK/8 = 2.5MHz => 2.5 ticks/us */
  uint32_t ticks = ((uint32_t)us * 25u) / 10u;
  if (ticks > (uint32_t)PR2) ticks = PR2;
  return (uint16_t)ticks;
}

static void ServoSet_OC1_us(uint16_t us) { OC1RS = UsToOCrs(ClampGrabUs(us)); }
static void ServoSet_OC4_us(uint16_t us) { OC4RS = UsToOCrs(ClampArmUs(us)); }

/*====================== SERVO RAMP ENGINE =====================*/
static volatile uint16_t grab_cur_us = GRAB_OPEN_US;
static volatile uint16_t grab_tgt_us = GRAB_OPEN_US;
static volatile uint16_t arm_cur_us  = ARM_BUCKET_READY_US;
static volatile uint16_t arm_tgt_us  = ARM_BUCKET_READY_US;

static uint8_t ServoRampTimerNum = 15u; /* TIMER15_RESP_FUNC -> PostCollectService */
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

/*====================== SERVO PWM INIT =====================*/
static void InitServoPWM(void)
{
  /* Timer2 shared servo timebase */
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TCKPS = SERVO_TIMER_PRESCALE_BITS; /* 1:8 */
    TMR2 = 0;
    PR2 = (uint16_t)SERVO_PR2_VALUE;
    T2CONbits.ON = 1;
  }

  /* Make sure pins are digital outputs */
#ifdef _ANSELB_ANSB4_MASK
  ANSELBCLR = _ANSELB_ANSB4_MASK; /* RB4 digital */
#endif
#ifdef _ANSELA_ANSA4_MASK
  ANSELACLR = _ANSELA_ANSA4_MASK; /* RA4 digital */
#endif
  TRISBbits.TRISB4 = 0;
  TRISAbits.TRISA4 = 0;

  /* ===== OC1 (grabber) ===== */
  OC1CON = 0;
  OC1CONbits.OCTSEL = 0;     /* Timer2 */
  OC1R  = UsToOCrs(GRAB_OPEN_US);  /* initialize BOTH registers */
  OC1RS = UsToOCrs(GRAB_OPEN_US);
  OC1CONbits.OCM = 0b110;    /* PWM mode */
  OC1CONbits.ON = 1;

  /* ===== OC4 (arm) ===== */
  OC4CON = 0;
  OC4CONbits.OCTSEL = 0;     /* Timer2 */
  OC4R  = UsToOCrs(ARM_BUCKET_READY_US);
  OC4RS = UsToOCrs(ARM_BUCKET_READY_US);
  OC4CONbits.OCM = 0b110;
  OC4CONbits.ON = 1;

  /*
    ===== PPS mapping =====
    YOU MUST ENSURE THESE CODES MATCH YOUR CHIP.
    If RB4/RA4 outputs still don't move, the PPS codes are wrong.

    Common PIC32MX mapping often is:
      0b0101 = OC1
      0b1000 = OC4
    But your project previously used different codes for OC3/OC5, so verify.
  */
#ifndef PPS_OC1_CODE
#define PPS_OC1_CODE 0b0101
#endif
#ifndef PPS_OC4_CODE
#define PPS_OC4_CODE 0b1000
#endif

  RPB4Rbits.RPB4R = PPS_OC1_CODE;  /* OC1 -> RB4 */
  RPA4Rbits.RPA4R = PPS_OC4_CODE;  /* OC4 -> RA4 */

  ServoInitDone = true;
  DB_printf("CollectService: PWM init ok. Timer2=%uHz PR2=%u. PPS OC1=%u OC4=%u\r\n",
            (unsigned)SERVO_HZ, (unsigned)PR2, (unsigned)PPS_OC1_CODE, (unsigned)PPS_OC4_CODE);
}

/*====================== HIGH-LEVEL ACTIONS =====================*/
static void ArmToReady(void)          { ServoRampTo(grab_tgt_us, ARM_BUCKET_READY_US); }
static void ArmDownToDispenser(void)  { ServoRampTo(grab_tgt_us, ARM_DOWN_DISPENSER_US); }
static void ArmUpToBucket(void)       { ServoRampTo(grab_tgt_us, ARM_BUCKET_READY_US); }
static void GrabberOpen(void)         { ServoRampTo(GRAB_OPEN_US,  arm_tgt_us); }
static void GrabberClose(void)        { ServoRampTo(GRAB_CLOSE_US, arm_tgt_us); }

/*====================== STATE HELPERS =====================*/
static void RequestNudge(uint8_t cmdByte)
{
  ES_Event_t CmdEvent = { ES_CMD_REQ, cmdByte };
  PostSPILeaderService(CmdEvent);
}

static void TransitionTo(CollectState_t next, uint16_t tMs)
{
  CurState = next;

  switch (next)
  {
    case COLLECT_IDLE:
      ES_Timer_StopTimer(COLLECT_TIMER);
      break;

    case COLLECT_GO_READY:
      ArmToReady();
      GrabberOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_READY_SETTLE_MS : tMs);
      break;

    case COLLECT_ARM_DOWN:
      ArmDownToDispenser();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_ARM_MOVE_MS : tMs);
      break;

    case COLLECT_GRAB_CLOSE:
      GrabberClose();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_GRAB_MOVE_MS : tMs);
      break;

    case COLLECT_NUDGE_BACK:
      RequestNudge(CMD_NUDGE_BACK);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
      break;

    case COLLECT_ARM_UP_BUCKET:
      ArmUpToBucket();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_ARM_MOVE_MS : tMs);
      break;

    case COLLECT_GRAB_OPEN:
      GrabberOpen();
      ES_Timer_InitTimer(COLLECT_TIMER, (tMs == 0u) ? T_RELEASE_MOVE_MS : tMs);
      break;

    case COLLECT_NUDGE_FWD:
      RequestNudge(CMD_NUDGE_FWD);
      ES_Timer_InitTimer(COLLECT_TIMER, T_NUDGE_MS);
      break;

    default:
      ES_Timer_StopTimer(COLLECT_TIMER);
      break;
  }
}

/*=========================== PUBLIC API =============================*/
bool InitCollectService(uint8_t Priority)
{
  MyPriority = Priority;
  CurState = COLLECT_IDLE;
  BallCount = 0u;

  if (!ServoInitDone) InitServoPWM();

  grab_cur_us = GRAB_OPEN_US;  grab_tgt_us = GRAB_OPEN_US;
  arm_cur_us  = ARM_BUCKET_READY_US; arm_tgt_us = ARM_BUCKET_READY_US;
  ServoWriteAll();

  ES_Timer_StopTimer(COLLECT_TIMER);
  ES_Timer_StopTimer(ServoRampTimerNum);

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

  /* ramp timer tick */
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
      if ((ThisEvent.EventType == ES_START_BUTTON) || (ThisEvent.EventType == ES_COLLECT_START))
      {
        BallCount = 0u;
        whichCollect = (ThisEvent.EventType == ES_COLLECT_START) ? ThisEvent.EventParam : FIRST_COLLECT;
        TransitionTo(COLLECT_GO_READY, 0u);
      }
      break;

    case COLLECT_GO_READY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_ARM_DOWN, 0u);
      break;

    case COLLECT_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_GRAB_CLOSE, 0u);
      break;

    case COLLECT_GRAB_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_NUDGE_BACK, 0u);
      break;

    case COLLECT_NUDGE_BACK:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_ARM_UP_BUCKET, 0u);
      break;

    case COLLECT_ARM_UP_BUCKET:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_GRAB_OPEN, 0u);
      break;

    case COLLECT_GRAB_OPEN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_NUDGE_FWD, 0u);
      break;

    case COLLECT_NUDGE_FWD:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == COLLECT_TIMER))
        TransitionTo(COLLECT_CHECK_COUNT, 0u);
      break;

    case COLLECT_CHECK_COUNT:
      BallCount++;
      if (BallCount < BALL_TARGET_COUNT) TransitionTo(COLLECT_ARM_DOWN, 0u);
      else TransitionTo(COLLECT_DONE, 0u);
      break;

    case COLLECT_DONE:
    {
      /* safe pose */
      GrabberOpen();
      ArmToReady();

      /* SPI notify */
      ES_Event_t doneEvt = { ES_CMD_REQ, CMD_FIRST_COLLECT_DONE };
      switch (whichCollect) {
        case FIRST_COLLECT:  doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
        case SECOND_COLLECT: doneEvt.EventParam = CMD_SECOND_COLLECT_DONE; break;
        case OTHER_COLLECT:  doneEvt.EventParam = CMD_OTHER_COLLECT_DONE;  break;
        default:             doneEvt.EventParam = CMD_FIRST_COLLECT_DONE;  break;
      }
      PostSPILeaderService(doneEvt);

      /* ===== IMPORTANT: COMMUNICATE TO DISPENSE ===== */
      ES_Event_t cd = { ES_COLLECT_DONE, 0 };
      ES_PostAll(cd);

      TransitionTo(COLLECT_IDLE, 0u);
      break;
    }

    default:
      TransitionTo(COLLECT_IDLE, 0u);
      break;
  }

  return ReturnEvent;
}