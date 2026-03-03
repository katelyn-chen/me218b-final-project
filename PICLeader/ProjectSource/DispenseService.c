/****************************************************************************
 Module
   DispenseService.c

 Summary
   Dispense sequence using push-down arm and rotating bucket bottom.
   Bucket swing/ready position is commanded through CollectService (OC4)
   to avoid OC4 conflicts.

   Sequence:
     - On ES_DISPENSE_START:
         1) Lower push-down arm
         2) Small backup delay
         3) Rotate bucket bottom (dump)
         4) Wait for balls
         5) Raise push arm

   Also:
     - Field side indication flag servo (RB3) is controlled by temporarily
       re-mapping OC5 to RB3 at the beginning of the game, then mapping OC5
       back to RB6 for the push-down arm for the rest of the game.
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
#define T_PUSH_ARM_DOWN_MS       950u
#define T_BACKUP_DELAY_MS        100u
#define T_ROTATE_BUCKET_90_MS    200u   /* tuned 90 degree rotation */
#define T_ROTATE_BUCKET_180_MS   400u
#define T_DISPENSE_WAIT_MS       1200u
#define T_PUSH_ARM_UP_MS         1500u

/*=========================== SERVO POSITIONS ============================*/
/* OC5 : push-down arm (RB6) */
#define US_PUSH_ARM_UP           2500u
#define US_PUSH_ARM_DOWN         600u   /* tuned */

/* OC3 : continuous rotation bucket bottom (RB10) */
#define US_BUCKET_STOP           1400u  /* tuned */
#define US_BUCKET_ROTATE_CW      1700u

/*============================== TIMER ==============================*/
#ifndef DISPENSE_TIMER
#define DISPENSE_TIMER           2u
#endif

/* We use a spare ES timer just to "hold flag pulses" briefly */
#ifndef FLAG_HOLD_TIMER
#define FLAG_HOLD_TIMER          14u    /* TIMER14_RESP_FUNC must PostDispenseService */
#endif
#define T_FLAG_HOLD_MS           900u   /* servo needs some time to reach target */

/*============================== STATE ==============================*/
typedef enum {
  DISP_IDLE = 0,
  DISP_PUSH_ARM_DOWN,
  DISP_BACKUP_DELAY,
  DISP_ROTATE_BUCKET,
  DISP_WAIT_DROP,
  DISP_PUSH_ARM_UP,
  DISP_DONE
} DispState_t;

static DispState_t CurState;
static uint8_t MyPriority;
static bool FirstDispense = true;

/*====================== PWM HELPERS =====================*/
static bool ServoInitDone = false;

static void InitServoPWM(void);
static uint16_t UsToOCrs(uint16_t us);

static void Servo_OC3(uint16_t us); /* bucket bottom */
static void Servo_OC4(uint16_t us); /* bucket arm (owned by CollectService, but helper exists) */
static void Servo_OC5(uint16_t us); /* push-down arm */

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
  FLAG SERVO (RB3) — OC5 SHARE VERSION (no interrupts)
  - The flag servo is only needed once at the beginning when we determine
    BLUE/GREEN side, then it should just stay there.
  - We share OC5 between:
      * RB3 (flag signal)   -- only for side indication
      * RB6 (push-down arm) -- used during dispense for the rest of the game

  How it works:
    - When ES_INDICATE_SIDE arrives:
        1) remap OC5 to RB3 (unmap RB6)
        2) re-init OC5 and write the flag pulse width
        3) hold pulses for ~0.9s so servo can actually move
        4) on FLAG_HOLD_TIMER timeout, remap OC5 back to RB6 and re-init OC5
           so the push-down arm works normally again
======================================================================*/

/* Flag pulse widths (standard-ish servo values, tune if needed) */
#define US_FLAG_GREEN           900u
#define US_FLAG_BLUE            2100u
#define US_FLAG_CENTER          1500u

/* Only set flag once at the start (ignore future side events) */
static bool FlagLatched = false;

/* forward decls */
static void SwitchOC5ToFlag(void);
static void SwitchOC5ToArm(void);
static void OC5WriteUs(uint16_t us);

/* write pulse width to OC5RS (Timer2 base) */
static void OC5WriteUs(uint16_t us)
{
  OC5RS = UsToOCrs(us);
}

/* OC5 -> RB3 (flag) */
static void SwitchOC5ToFlag(void)
{
  DB_printf("FLAG: switching OC5 -> RB3 (flag)\r\n");

  /* unmap arm pin first so we never drive two pins */
#ifdef RPB6Rbits
  RPB6Rbits.RPB6R = 0;
#endif

  /* RB3 as digital output */
  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB3 = 0;
  LATBbits.LATB3 = 0;

  /* map OC5 to RB3 */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0b0110; /* OC5 */
#endif

  /* re-init OC5 (TA style: clear + reconfigure) */
  OC5CON = 0;
  OC5R   = UsToOCrs(US_FLAG_CENTER);
  OC5RS  = UsToOCrs(US_FLAG_CENTER);
  OC5CONbits.OCM    = 0b110; /* PWM mode */
  OC5CONbits.OCTSEL = 0;     /* Timer2 */
  OC5CONbits.ON     = 1;
}

/* OC5 -> RB6 (push-down arm) */
static void SwitchOC5ToArm(void)
{
  DB_printf("FLAG: switching OC5 -> RB6 (arm)\r\n");

  /* unmap flag pin */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R = 0;
#endif

  /* RB6 as output */
  TRISBbits.TRISB6 = 0;
  LATBbits.LATB6 = 0;

  /* map OC5 to RB6 */
#ifdef RPB6Rbits
  RPB6Rbits.RPB6R = 0b0110; /* OC5 */
#endif

  /* re-init OC5 for arm values */
  OC5CON = 0;
  OC5R   = UsToOCrs(US_PUSH_ARM_UP);
  OC5RS  = UsToOCrs(US_PUSH_ARM_UP);
  OC5CONbits.OCM    = 0b110;
  OC5CONbits.OCTSEL = 0;
  OC5CONbits.ON     = 1;
}

/*=========================== INIT ============================*/
bool InitDispenseService(uint8_t Priority)
{
  MyPriority = Priority;

  if (!ServoInitDone) InitServoPWM();

  /* default at boot: OC5 controls ARM on RB6, flag will be set once later */
  SwitchOC5ToArm();
  FlagLatched = false;

  BucketRotateStop();
  PushArmUp();

  CurState = DISP_IDLE;
  FirstDispense = true;

  ES_Timer_StopTimer(DISPENSE_TIMER);
  ES_Timer_StopTimer(FLAG_HOLD_TIMER);

  DB_printf("DispenseService: init done\r\n");

  ES_Event_t e = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, e);
}

bool PostDispenseService(ES_Event_t e)
{
  return ES_PostToService(MyPriority, e);
}

/*=========================== STATE MACHINE ============================*/
ES_Event_t RunDispenseService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  /* after holding flag pulses long enough, give OC5 back to the arm */
  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == FLAG_HOLD_TIMER))
  {
    SwitchOC5ToArm();
    return ReturnEvent;
  }

  /* side indication can come at any time since keystroke is ES_PostAll */
  if (ThisEvent.EventType == ES_INDICATE_SIDE)
  {
    Field_t side = (Field_t)ThisEvent.EventParam;
    DB_printf("Side indicated! side=%d\r\n", side);

    /* only do this once at the start */
    if (FlagLatched) {
      DB_printf("FLAG already set, ignoring\r\n");
      return ReturnEvent;
    }
    FlagLatched = true;

    SwitchOC5ToFlag();

    if (side == FIELD_BLUE) {
      OC5WriteUs(US_FLAG_BLUE);
      DB_printf("FLAG -> BLUE\r\n");
    } else if (side == FIELD_GREEN) {
      OC5WriteUs(US_FLAG_GREEN);
      DB_printf("FLAG -> GREEN\r\n");
    } else {
      OC5WriteUs(US_FLAG_CENTER);
      DB_printf("FLAG -> CENTER\r\n");
    }

    /* keep pulses for a bit so servo can reach target before we swap back */
    ES_Timer_InitTimer(FLAG_HOLD_TIMER, T_FLAG_HOLD_MS);
    return ReturnEvent;
  }

  switch (CurState)
  {
    case DISP_IDLE:
      if (ThisEvent.EventType == ES_DISPENSE_START)
      {
        DB_printf("Dispense start\r\n");
        PushArmDown();

#ifdef CMD_DISPENSE_START
        RequestCmd(CMD_DISPENSE_START);
#endif

        CurState = DISP_PUSH_ARM_DOWN;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_PUSH_ARM_DOWN_MS);
      }
      break;

    case DISP_PUSH_ARM_DOWN:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_BACKUP_DELAY;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_BACKUP_DELAY_MS);
      }
      break;

    case DISP_BACKUP_DELAY:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketRotateStart();

        /* first dispense vs second dispense logic */
        if (FirstDispense) {
          ES_Timer_InitTimer(DISPENSE_TIMER, T_ROTATE_BUCKET_90_MS);
          FirstDispense = false;  /* (bug fix) was '==' before */
        } else {
          ES_Timer_InitTimer(DISPENSE_TIMER, T_ROTATE_BUCKET_180_MS);
          FirstDispense = true;   /* (bug fix) was '==' before */
        }

        CurState = DISP_ROTATE_BUCKET;
      }
      break;

    case DISP_ROTATE_BUCKET:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketRotateStop();
        CurState = DISP_WAIT_DROP;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_DISPENSE_WAIT_MS);
      }
      break;

    case DISP_WAIT_DROP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        PushArmUp();
        CurState = DISP_PUSH_ARM_UP;
        ES_Timer_InitTimer(DISPENSE_TIMER, T_PUSH_ARM_UP_MS);
      }
      break;

    case DISP_PUSH_ARM_UP:
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        CurState = DISP_DONE;

        /* close bucket, ready for next collect */
        BucketRotateStart();
        ES_Timer_InitTimer(DISPENSE_TIMER, T_ROTATE_BUCKET_90_MS);
      }
      break;

    case DISP_DONE:
    {
      /* we use a final timeout to stop the bucket close motion */
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == DISPENSE_TIMER))
      {
        BucketRotateStop();
      }

      DB_printf("Dispense complete\r\n");

#ifdef CMD_DISPENSE_DONE
      RequestCmd(CMD_DISPENSE_DONE);
#endif

      ES_Event_t done = { ES_DISPENSE_COMPLETE, 0 };
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
  /* Timer2 init only if OFF (shared timebase) */
  if (T2CONbits.ON == 0u)
  {
    T2CON = 0;
    T2CONbits.TCKPS = 0b011u; /* 1:8 prescale */
    TMR2 = 0;
    PR2 = 49999u;             /* 20ms at PBCLK=20MHz prescale 1:8 */
    T2CONbits.ON = 1;
  }

  /* make sure these pins are digital outputs */
  ANSELBbits.ANSB3  = 0;
  TRISBbits.TRISB3  = 0;
  TRISBbits.TRISB6  = 0;
  TRISBbits.TRISB10 = 0;

  /* OC3 bottom */
  OC3CON = 0; OC3R = 0; OC3RS = 0;
  OC3CONbits.OCTSEL = 0;
  OC3CONbits.OCM    = 0b110;
  OC3CONbits.ON     = 1;

  /* OC5 is shared between arm+flag, but initialize it once here */
  OC5CON = 0; OC5R = 0; OC5RS = 0;
  OC5CONbits.OCTSEL = 0;
  OC5CONbits.OCM    = 0b110;
  OC5CONbits.ON     = 1;

  /* PPS mapping: OC3 stays on RB10 always; OC5 gets swapped later as needed */
  RPB10Rbits.RPB10R = 0b0101; /* OC3 -> RB10 */

  /* default OC5 -> RB6 */
  RPB6Rbits.RPB6R   = 0b0110; /* OC5 -> RB6 */
#ifdef RPB3Rbits
  RPB3Rbits.RPB3R   = 0;      /* ensure RB3 is detached at init */
#endif

  ServoInitDone = true;
}

static uint16_t UsToOCrs(uint16_t us)
{
  /* PBCLK=20MHz, Timer2 prescale=1:8 => tick=0.4us => 2.5 ticks/us */
  return (uint16_t)((us * 25u) / 10u);
}

static void Servo_OC3(uint16_t us){ OC3RS = UsToOCrs(us); }
static void Servo_OC4(uint16_t us){ OC4RS = UsToOCrs(us); }
static void Servo_OC5(uint16_t us){ OC5RS = UsToOCrs(us); }