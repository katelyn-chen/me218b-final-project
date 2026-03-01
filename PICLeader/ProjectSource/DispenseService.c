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
/* OC5 : push-down arm (RB6 in your PPS) */
#define US_PUSH_ARM_UP        1200u
#define US_PUSH_ARM_DOWN      2000u

/* OC3 : continuous rotation bucket bottom (RB10 on your other code; keep OC3 here) */
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

static DispState_t CurState = DISP_IDLE;
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
  FLAG SERVO (RB3) — SOFTWARE SERVO OUTPUT (Timer4 ISR)
  IMPORTANT:
    - RB3 MUST NOT be PPS-mapped to any OC output.
    - Flag only changes on ES_INDICATE_SIDE event.
======================================================================*/
#define US_FLAG_BLUE          2000u
#define US_FLAG_GREEN         1000u
#define US_FLAG_CENTER        1500u

#define SERVO_PERIOD_US       20000u

#define FLAG_TRIS             TRISBbits.TRISB3
#define FLAG_LAT              LATBbits.LATB3
#define FLAG_ANSEL            ANSELBbits.ANSB3

#define T4_PRESCALE_BITS      0b11u // 1:256 prescale
#define TICKS_FOR_US(us)      ((uint16_t)(((uint32_t)(us) * 25u) / 10u))
//#define PBCLK 40000000UL
//#define PRESCALE 256UL
//#define TICKS_FOR_US(us) ((uint16_t)(((us) * (PBCLK / PRESCALE)) / 1000000UL))

typedef enum { FLAG_PHASE_START = 0, FLAG_PHASE_END = 1 } FlagPhase_t;
static volatile FlagPhase_t FlagPhase = FLAG_PHASE_START;
static volatile uint16_t FlagPulseUs = US_FLAG_CENTER;
static bool FlagInitDone = false;

static void InitFlagServoRB3(void);
static void FlagSetPulseUs(uint16_t us);

static void InitFlagServoRB3(void)
{
  if (FlagInitDone) return;

  FLAG_ANSEL = 0;
  FLAG_TRIS  = 0;
  FLAG_LAT   = 0;

#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0;   /* DETACH PPS from RB3 so GPIO works */
#endif

  INTCONbits.MVEC = 1;

  T4CON = 0;
  T4CONbits.TCS   = 0;
  T4CONbits.TCKPS = T4_PRESCALE_BITS;
  TMR4 = 0;

  IPC4bits.T4IP = 4;
  IPC4bits.T4IS = 0;

  IFS0CLR = _IFS0_T4IF_MASK;
  IEC0SET = _IEC0_T4IE_MASK;

  FlagPhase = FLAG_PHASE_START;
  PR4 = 1;
//  PR4  = TICKS_FOR_US(1000);
  TMR4 = 0;

  T4CONbits.ON = 1;

  FlagInitDone = true;
}

static void FlagSetPulseUs(uint16_t us)
{
  if (us < 800u)  us = 800u;
  if (us > 2200u) us = 2200u;

  __builtin_disable_interrupts();
  FlagPulseUs = us;
  __builtin_enable_interrupts();
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) T4Handler(void)
{
//  DB_printf("0\r\n");
    IFS0CLR = _IFS0_T4IF_MASK;

  if (FlagPhase == FLAG_PHASE_START)
  {
    FLAG_LAT = 1;
    FlagPhase = FLAG_PHASE_END;

    TMR4 = 0;
    PR4  = TICKS_FOR_US(FlagPulseUs);
  }
  else
  {
    FLAG_LAT = 0;
    FlagPhase = FLAG_PHASE_START;

    uint16_t remainUs = (FlagPulseUs >= SERVO_PERIOD_US) ? 1u
                      : (uint16_t)(SERVO_PERIOD_US - FlagPulseUs);

    TMR4 = 0;
    PR4  = TICKS_FOR_US(remainUs);
  }
}

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  InitFlagServoRB3();
  FlagSetPulseUs(US_FLAG_CENTER);

  BucketRotateStop();
  PushArmUp();

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

      if(ThisEvent.EventType == ES_INDICATE_SIDE)
      {
        Field_t side = (Field_t)ThisEvent.EventParam;
        DB_printf("Side indicated! Moving flag in dispense. side = %d\r\n", side);

        if (side == FIELD_BLUE) {
          FlagSetPulseUs(US_FLAG_BLUE);
        } else if (side == FIELD_GREEN) {
          FlagSetPulseUs(US_FLAG_GREEN);
        } else {
          FlagSetPulseUs(US_FLAG_CENTER);
        }
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

  #ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0;   /* RB3 must stay GPIO */
#endif
ANSELBbits.ANSB3 = 0;
TRISBbits.TRISB3 = 0;

  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB3 = 0;

  /* OC3 bottom */
  OC3CON=0; OC3R=0; OC3RS=0;
  OC3CONbits.OCTSEL=0; OC3CONbits.OCM=0b110; OC3CONbits.ON=1;

  /* OC5 push-down arm */
  OC5CON=0; OC5R=0; OC5RS=0;
  OC5CONbits.OCTSEL=0; OC5CONbits.OCM=0b110; OC5CONbits.ON=1;

  /* PPS mapping: keep these consistent with your wiring */
  TRISBbits.TRISB6 = 0;
  TRISBbits.TRISB10 = 0;

  RPB6Rbits.RPB6R   = 0b0110; /* OC5 -> RB6 */
  RPB10Rbits.RPB10R = 0b0101; /* OC3 -> RB10 */

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  return (uint16_t)((us*25u)/10u);
}

static void Servo_OC3(uint16_t us){ OC3RS=UsToOCrs(us); }
static void Servo_OC4(uint16_t us){ OC4RS=UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS=UsToOCrs(us); }