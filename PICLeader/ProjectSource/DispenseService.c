/****************************************************************************
 Module
   DispenseService.c

 Summary
   Dispense sequence using push-down arm and rotating bucket bottom.
   Bucket swing/ready position is commanded through CollectService (OC4)
   to avoid OC4 conflicts.

   Sequence:
     - On ES_WAIT_BALL: request bucket/arm to go to collect-ready (via CollectService)
     - On ES_DISPENSE_START:
         1) Lower push-down arm
         2) Small backup delay
         3) Rotate bucket bottom (dump)
         4) Wait for balls
         5) Raise push arm
****************************************************************************/

#include "DispenseService.h"
#include "SPILeaderService.h"
#include "CollectService.h"   /* for PostCollectService */

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*============================== TIMING ==============================*/
#define T_PUSH_ARM_DOWN_MS     950u
#define T_BACKUP_DELAY_MS      100u
#define T_ROTATE_BUCKET_MS     900u
#define T_DISPENSE_WAIT_MS     1200u
#define T_PUSH_ARM_UP_MS       1500u

/*=========================== SERVO POSITIONS ============================*/
/* OC5 : push-down arm (RB6) */
#define US_PUSH_ARM_UP        2500u
#define US_PUSH_ARM_DOWN      600u // tuned

/* OC3 : continuous rotation bucket bottom (RB10) */
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
  DISP_ROTATE_BUCKET,
  DISP_WAIT_DROP,
  DISP_PUSH_ARM_UP,
  DISP_DONE
} DispState_t;

static DispState_t CurState;
static uint8_t MyPriority;

/*====================== PWM HELPERS =====================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);

static void Servo_OC3(uint16_t us); // bucket bottom
static void Servo_OC4(uint16_t us); // bucket arm
static void Servo_OC5(uint16_t us); // push-down arm

/*====================== ACTION HELPERS =====================*/
static void PushArmDown(void){ Servo_OC5(US_PUSH_ARM_DOWN); }
static void PushArmUp(void){   Servo_OC5(US_PUSH_ARM_UP);   }

static void BucketRotateStart(void){ Servo_OC3(US_BUCKET_ROTATE_CW); }
static void BucketRotateStop(void){  Servo_OC3(US_BUCKET_STOP);      }

/*====================== SPI HELPERS =====================*/
static void RequestCmd(uint8_t cmdByte)
{
  ES_Event_t CmdEvent;
  CmdEvent.EventType  = ES_CMD_REQ;
  CmdEvent.EventParam = cmdByte;
  PostSPILeaderService(CmdEvent);
}

/*======================================================================
  FLAG SERVO (RB3) — trying to share OC5 via PPS swap
  - Goal: move flag ONCE at beginning when side is determined
  - Then give OC5 back to push-down arm (RB6) for the rest of the game

  How it works:
    - On ES_INDICATE_SIDE:
        * PPS-map OC5 -> RB3 (flag signal)
        * write OC5RS to desired pulse width
        * start a short timer so we keep pulses for a bit (servo can reach)
    - On timeout:
        * PPS-map OC5 -> RB6 (arm signal)
        * detach RB3 PPS (flag stops getting pulses; should stay in place)
======================================================================*/

/* so here jsut hold OC5 on RB3" briefly */
#ifndef FLAG_HOLD_TIMER
#define FLAG_HOLD_TIMER        14u  
#endif
#define T_FLAG_HOLD_MS         1800u 

/* flag servo pulse widths (tune) */
#define US_FLAG_GREEN          900u
#define US_FLAG_BLUE           2000u
#define US_FLAG_CENTER         1500u


/* track where OC5 is currently routed */
typedef enum {
  OC5_TO_ARM = 0,
  OC5_TO_FLAG = 1
} OC5Route_t;

static OC5Route_t OC5Route = OC5_TO_ARM;

/* PPS swap helpers */
static void RouteOC5ToFlagRB3(void)
{
  /* make RB3 a clean digital output */
  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB3 = 0;

  /* IMPORTANT: detach OC5 from RB6 so only one pin is driven */
#ifdef RPB6Rbits
  RPB6Rbits.RPB6R = 0;
#endif

  /* OC5 -> RB3 */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0b0110;  
#endif

  OC5Route = OC5_TO_FLAG;
  DB_printf("FLAG: OC5 routed to RB3\r\n");
}

static void RouteOC5ToArmRB6(void)
{
  /* detach OC5 from RB3 */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0;
#endif

  /* OC5 -> RB6 (push-down arm pin) */
#ifdef RPB6Rbits
  RPB6Rbits.RPB6R = 0b0110;   /* OC5 -> RB6 */
#endif

  OC5Route = OC5_TO_ARM;
  DB_printf("FLAG: OC5 routed back to RB6\r\n");
}

static void FlagCommandPulseUs(uint16_t us)
{
  /* clamp-ish (avoid crazy) */
  if (us < 700u)  us = 700u;
  if (us > 2300u) us = 2300u;

  OC5RS = UsToOCrs(us);
}

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  /* default: OC5 routed to arm (RB6) at boot */
  RouteOC5ToArmRB6();

  BucketRotateStop();
  PushArmUp();
  CurState = DISP_IDLE;

  ES_Timer_StopTimer(DISPENSE_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { ES_INIT, 0 };
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
    if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == FLAG_HOLD_TIMER))
  {
    /* after holding long enough, give OC5 back to the arm */
    RouteOC5ToArmRB6();
    return ReturnEvent;
  }
  /* your keystroke posts the event to all services (ES_PostAll) and you handle it in every state (you currently don’t). */
    if (ThisEvent.EventType == ES_INDICATE_SIDE)
  {
    Field_t side = (Field_t)ThisEvent.EventParam;
    DB_printf("Side indicated (global)! Setting flag using OC5 PPS swap. side=%d\r\n", side);

    /* route OC5 to RB3 (flag), command pulse, hold briefly */
    RouteOC5ToFlagRB3();

    if (side == FIELD_BLUE) {
      FlagCommandPulseUs(US_FLAG_BLUE);
      DB_printf("FLAG -> BLUE\r\n");
    } else if (side == FIELD_GREEN) {
      FlagCommandPulseUs(US_FLAG_GREEN);
      DB_printf("FLAG -> GREEN\r\n");
    } else {
      FlagCommandPulseUs(US_FLAG_CENTER);
      DB_printf("FLAG -> CENTER\r\n");
    }

    ES_Timer_InitTimer(FLAG_HOLD_TIMER, T_FLAG_HOLD_MS);
    return ReturnEvent;
  }

  switch(CurState)
  {
    case DISP_IDLE:
      if(ThisEvent.EventType == ES_DISPENSE_START)
      {
        DB_printf("Dispense start\r\n");
        PushArmDown();

#ifdef CMD_DISPENSE_START
        RequestCmd(CMD_DISPENSE_START);
#endif

        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_PUSH_ARM_DOWN_MS);
      }

      if(ThisEvent.EventType == ES_WAIT_BALL)
      {
        /* Request bucket/arm to be at ready position via CollectService (OC4 owner) */
        ES_Event_t e;
        e.EventType = ES_BUCKET_READY;
        e.EventParam = 0;
        PostCollectService(e);
        DB_printf("requested bucket ready via CollectService\r\n");
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
        BucketRotateStart();
        CurState = DISP_ROTATE_BUCKET;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_ROTATE_BUCKET_MS);
      }
      break;

    case DISP_ROTATE_BUCKET:
      if((ThisEvent.EventType==ES_TIMEOUT) && (ThisEvent.EventParam==DISPENSE_TIMER))
      {
        BucketRotateStop();
        CurState = DISP_WAIT_DROP;
        ES_Timer_InitTimer(DISPENSE_TIMER,T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP:
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
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCKPS = 0b011u;
    TMR2 = 0;
    PR2 = 49999u;
    T2CONbits.ON = 1;
  }

  // flag indicator servo
  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB3 = 0;
  TRISBbits.TRISB6 = 0;
  TRISBbits.TRISB10 = 0;

  /* OC3 bottom */
  OC3CON=0; OC3R=0; OC3RS=0;
  OC3CONbits.OCTSEL=0; OC3CONbits.OCM=0b110;

  /* OC5 push-down arm */
  OC5CON=0; OC5R=0; OC5RS=0;
  OC5CONbits.OCTSEL=0; OC5CONbits.OCM=0b110;

  RPB6Rbits.RPB6R   = 0b0110; /* OC5 -> RB6 */
  RPB10Rbits.RPB10R = 0b0101; /* OC3 -> RB10 */
  OC3CONbits.ON=1; OC5CONbits.ON=1;

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us*25u)/10u);
}

static void Servo_OC3(uint16_t us){ OC3RS=UsToOCrs(us); }
static void Servo_OC4(uint16_t us){ OC4RS=UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS=UsToOCrs(us); }