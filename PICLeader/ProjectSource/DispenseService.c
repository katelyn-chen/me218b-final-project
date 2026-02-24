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

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

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

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  InitServoPWM();

  BucketRotateStop();
  PushArmUp();
  BucketToCollect();

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
        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_DOWN_MS);
      }
      break;

    case DISP_PUSH_ARM_DOWN:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        CurState = DISP_BACKUP_DELAY;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_BACKUP_DELAY_MS);
      }
      break;

    case DISP_BACKUP_DELAY:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        BucketToDispense();
        CurState = DISP_MOVE_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_BUCKET_MOVE_MS);
      }
      break;

    case DISP_MOVE_BUCKET:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        BucketRotateStart();
        CurState = DISP_ROTATE_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_ROTATE_BUCKET_MS);
      }
      break;

    case DISP_ROTATE_BUCKET:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        BucketRotateStop();
        BucketQuarterTurns++;
        CurState = DISP_WAIT_DROP;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        BucketToCollect();
        CurState = DISP_RETURN_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_RETURN_MS);
      }
      break;

    case DISP_RETURN_BUCKET:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        PushArmUp();
        CurState = DISP_PUSH_ARM_UP;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_UP_MS);
      }
      break;

    case DISP_PUSH_ARM_UP:
      if(ThisEvent.EventType==ES_TIMEOUT)
      {
        CurState = DISP_DONE;
      }
      break;

    case DISP_DONE:
    {
        DB_printf("Dispense complete\r\n");

        ES_Event_t done = {ES_DISPENSE_COMPLETE,0};
        ES_PostAll(done);

        CurState = DISP_IDLE;
        break;
    }
  }

  return ReturnEvent;
}

/*=========================== PWM ============================*/
static void InitServoPWM(void)
{
  T2CON = 0;
  T2CONbits.TCKPS = 0b011;
  PR2 = 49999;
  TMR2 = 0;
  T2CONbits.ON = 1;

  OC3CON=0; OC3CONbits.OCM=0b110; OC3CONbits.ON=1;
  OC4CON=0; OC4CONbits.OCM=0b110; OC4CONbits.ON=1;
  OC5CON=0; OC5CONbits.OCM=0b110; OC5CONbits.ON=1;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us*25u)/10u);
}

static void Servo_OC3(uint16_t us){ OC3RS=UsToOCrs(us); }
static void Servo_OC4(uint16_t us){ OC4RS=UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS=UsToOCrs(us); }