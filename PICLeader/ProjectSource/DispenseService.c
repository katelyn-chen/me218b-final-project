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
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*============================== TIMING ==============================*/
#define T_PUSH_ARM_DOWN_MS     500u
#define T_BACKUP_DELAY_MS      100u   /* robot backing slightly */
#define T_BUCKET_MOVE_MS       600u
#define T_ROTATE_BUCKET_MS     400u
#define T_DISPENSE_WAIT_MS     1000u
#define T_RETURN_MS            600u
#define T_PUSH_ARM_UP_MS       500u

/*=========================== SERVO POSITIONS ============================*/
/* OC5 : push-down arm */
#define US_PUSH_ARM_UP        1200u
#define US_PUSH_ARM_DOWN      2000u

/* OC4 : bucket swing arm (collector <-> dispense) */
#define US_BUCKET_COLLECT     1200u
#define US_BUCKET_DISPENSE    2000u

/* OC3 : continuous rotation bucket bottom */
#define US_BUCKET_STOP        1500u
#define US_BUCKET_ROTATE_CW   1700u

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

static void Servo_OC3(uint16_t us);
static void Servo_OC4(uint16_t us);
static void Servo_OC5(uint16_t us);

/*====================== ACTION HELPERS =====================*/
static void PushArmDown(void){ Servo_OC5(US_PUSH_ARM_DOWN); }
static void PushArmUp(void){ Servo_OC5(US_PUSH_ARM_UP); }

static void BucketToDispense(void){ Servo_OC4(US_BUCKET_DISPENSE); }
static void BucketToCollect(void){ Servo_OC4(US_BUCKET_COLLECT); }

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

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  BucketQuarterTurns = 0u;

  BucketRotateStop();
  PushArmUp();
  BucketToCollect();

  ES_Timer_StopTimer(DISPENSE_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { ES_INIT,0 };
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

        /*
          Optional hook:
            request follower-side behavior at start of dispense if a command exists.
            Define CMD_DISPENSE_START in the SPI header if desired.
        */
#ifdef CMD_DISPENSE_START
        RequestCmd(CMD_DISPENSE_START);
#endif

        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_DOWN_MS);
      }
      if(ThisEvent.EventType == ES_INDICATE_SIDE)
      {
        Field_t side = ThisEvent.EventParam;
        DB_printf("Indicating side! %d\r\n", side);
        if (side == FIELD_BLUE) {
          PushArmDown();
        } else if (side == FIELD_GREEN) {
          PushArmUp();
        }
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

        /*
          Optional hook:
            request follower-side behavior at end of dispense if a command exists.
            Define CMD_DISPENSE_DONE in the SPI header if desired.
        */
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
  /* Timer2 as 50Hz timebase for OC3/OC4/OC5 PWM */
  T2CON = 0;
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = 0b011;   /* 1:8 */
  PR2 = 49999;
  TMR2 = 0;

  /* ensure OCs use Timer2 */
  OC3CON=0; OC3R=0; OC3RS=0; OC3CONbits.OCTSEL=0; OC3CONbits.OCM=0b110; OC3CONbits.ON=1;
  OC4CON=0; OC4R=0; OC4RS=0; OC4CONbits.OCTSEL=0; OC4CONbits.OCM=0b110; OC4CONbits.ON=1;
  OC5CON=0; OC5R=0; OC5RS=0; OC5CONbits.OCTSEL=0; OC5CONbits.OCM=0b110; OC5CONbits.ON=1;

  T2CONbits.ON = 1;

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us*25u)/10u);
}

static void Servo_OC3(uint16_t us){ OC3RS=UsToOCrs(us); }
static void Servo_OC4(uint16_t us){ OC4RS=UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS=UsToOCrs(us); }