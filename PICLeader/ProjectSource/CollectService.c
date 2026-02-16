#include "Collect.h"
#include "ES_Timers.h"
#include "ES_Configure.h"
#include "SPISM.h"
#include <stdbool.h>
#include <stdint.h>

// ------------------- params -------------------
#define FIRST   1
#define SECOND  2
#define OTHER   3

#define MIDDLE  4

// ------------------- tuning -------------------
#define BALL_TARGET_COUNT   6

#define T_APPROACH_MS       2000
#define T_GRAB_PREP_MS      400
#define T_GRAB_CLOSE_MS     350
#define T_DROP_MS           500
#define T_BACKUP_MS         800

// ------------------- follower SPI command words -------------------
// keep simple for now
#define OPCODE_DRIVE        0x1
#define OPCODE_STOP         0x2
#define DRIVE_FWD_SLOW      0x001
#define DRIVE_REV_SLOW      0x002

static void Follower_Stop(void) {
  SPISM_SendFollowerCmd((OPCODE_STOP << 12));
}

static void Follower_FwdSlow(void) {
  SPISM_SendFollowerCmd((OPCODE_DRIVE << 12) | DRIVE_FWD_SLOW);
}

static void Follower_RevSlow(void) {
  SPISM_SendFollowerCmd((OPCODE_DRIVE << 12) | DRIVE_REV_SLOW);
}

// ------------------- hardware hooks (Leader-owned) -------------------
static bool DispenserSensorTriggered(void);

static void GrabServoOpen(void);
static void GrabServoClose(void);

static void ArmPickPose(void);
static void ArmDropPose(void);
static void ArmTravelPose(void);

// Post function back to Navigate/HSM service
static void PostFindBucket(uint16_t param);

// ------------------- module state -------------------
static CSM_State_t CurrentState = CSM_IDLE;
static uint16_t CollectMode = OTHER;
static uint8_t BallCount = 0;

static void TransitionTo(CSM_State_t NextState, uint16_t timeMs);

void InitCollectSM(void) {
  CurrentState = CSM_IDLE;
  CollectMode = OTHER;
  BallCount = 0;

  Follower_Stop();
  ArmTravelPose();
  GrabServoOpen();
  ES_Timer_StopTimer(CollectTimer);
}

CSM_State_t GetCollectSMState(void) {
  return CurrentState;
}

ES_Event_t RunCollectSM(ES_Event_t ThisEvent) {
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (CurrentState) {

    case CSM_IDLE:
      if (ThisEvent.EventType == ES_ALIGN_COLLECT) {
        CollectMode = ThisEvent.EventParam;  // FIRST/SECOND/OTHER
        BallCount = 0;
        TransitionTo(CSM_APPROACH_DISPENSER, T_APPROACH_MS);
      }
      break;

    case CSM_APPROACH_DISPENSER:
      if (DispenserSensorTriggered()) {
        Follower_Stop();
        TransitionTo(CSM_GRAB_PREP, T_GRAB_PREP_MS);
      } else if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == CollectTimer)) {
        Follower_Stop();
        TransitionTo(CSM_GRAB_PREP, T_GRAB_PREP_MS);
      }
      break;

    case CSM_GRAB_PREP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == CollectTimer)) {
        TransitionTo(CSM_GRAB_CLOSE, T_GRAB_CLOSE_MS);
      }
      break;

    case CSM_GRAB_CLOSE:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == CollectTimer)) {
        TransitionTo(CSM_LIFT_AND_DROP, T_DROP_MS);
      }
      break;

    case CSM_LIFT_AND_DROP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == CollectTimer)) {
        BallCount++;
        TransitionTo(CSM_CHECK_COUNT, 0);
      }
      break;

    case CSM_CHECK_COUNT:
      if (BallCount < BALL_TARGET_COUNT) {
        TransitionTo(CSM_GRAB_PREP, T_GRAB_PREP_MS);
      } else {
        TransitionTo(CSM_BACK_OUT, T_BACKUP_MS);
      }
      break;

    case CSM_BACK_OUT:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == CollectTimer)) {
        Follower_Stop();
        ArmTravelPose();
        TransitionTo(CSM_DONE, 0);
      }
      break;

    case CSM_DONE:
      if (CollectMode == FIRST) {
        PostFindBucket(FIRST);
      } else {
        PostFindBucket(MIDDLE);
      }
      TransitionTo(CSM_IDLE, 0);
      break;

    default:
      TransitionTo(CSM_IDLE, 0);
      break;
  }

  return ReturnEvent;
}

static void TransitionTo(CSM_State_t NextState, uint16_t timeMs) {
  CurrentState = NextState;

  switch (NextState) {

    case CSM_IDLE:
      Follower_Stop();
      ArmTravelPose();
      GrabServoOpen();
      ES_Timer_StopTimer(CollectTimer);
      break;

    case CSM_APPROACH_DISPENSER:
      Follower_FwdSlow();
      ES_Timer_SetTimer(CollectTimer, timeMs);
      ES_Timer_StartTimer(CollectTimer);
      break;

    case CSM_GRAB_PREP:
      GrabServoOpen();
      ArmPickPose();
      ES_Timer_SetTimer(CollectTimer, timeMs);
      ES_Timer_StartTimer(CollectTimer);
      break;

    case CSM_GRAB_CLOSE:
      GrabServoClose();
      ES_Timer_SetTimer(CollectTimer, timeMs);
      ES_Timer_StartTimer(CollectTimer);
      break;

    case CSM_LIFT_AND_DROP:
      ArmDropPose();
      ES_Timer_SetTimer(CollectTimer, timeMs);
      ES_Timer_StartTimer(CollectTimer);
      break;

    case CSM_CHECK_COUNT:
      ES_Timer_StopTimer(CollectTimer);
      break;

    case CSM_BACK_OUT:
      Follower_RevSlow();
      ArmTravelPose();
      ES_Timer_SetTimer(CollectTimer, timeMs);
      ES_Timer_StartTimer(CollectTimer);
      break;

    case CSM_DONE:
      Follower_Stop();
      ES_Timer_StopTimer(CollectTimer);
      break;
  }
}
