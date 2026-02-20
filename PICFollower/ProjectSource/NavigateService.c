/****************************************************************************
 Module
   NavigateService.c

 Revision
   1.0.0

 Description
   Navigate State Machine
****************************************************************************/
#include "NavigateService.h"
#include "SPIFollowerService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

/*---------------------------- Module Defines -------------------------------*/

#define PBCLK_HZ            20000000u
#define PWM_FREQ_HZ         10000u

/* Timer2 @ 10kHz PWM with prescale 1:1 -> PR2 = 1999 */
#define T2_PRESCALE_BITS    (0b000u) /* 1:1 */
#define T2_PRESCALE_VAL     1u
#define PR2_VALUE           ((PBCLK_HZ / (T2_PRESCALE_VAL * PWM_FREQ_HZ)) - 1u)

/* duty presets (percent 0..100) */
#define DUTY_STOP           0u
#define DUTY_TRANS_HALF     50u
#define DUTY_TRANS_FULL     100u
#define DUTY_ROTATE         45u
#define DUTY_SEARCH         30u

/* rotate timing (ms) — will calibrate these on the floor */
// placeholders for now
#define ROTATE_45_TIME_MS   550u
#define ROTATE_90_TIME_MS   1100u

/* match with ES timer ID in ES_Configure.h */
#ifndef ROTATE_TIMER
#define ROTATE_TIMER MOTOR_TIMER
#endif

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  DEBUG,
  WAITING_FOR_SIDE,
  FIRST_COLLECT,
  COLLECT_ALIGN,
  FIRST_DISPENSE,
  INIT_FIND_MIDDLE,
  ALIGN_MID_BUCKET,
  INIT_FIND_END,
  ALIGN_END_BUCKET,
  INIT_REPEAT_COLLECT,
  FIND_COLLECT
} NavigateState_t;

typedef enum {
  IDLE,
  ULTRASONIC_ALIGN,
  BEACON_ALIGN,
  SIDE_FOUND
} InitOrientState_t;

typedef enum {
    LEFT,
    RIGHT
} RotateParam_t;

typedef enum {
    FIRST,
    MIDDLE,
    END
} BucketParam_t;


/*---------------------------- Module Variables --------------------------*/
static uint8_t      MyPriority;
static NavigateState_t curState;
static InitOrientState_t orientState;

/*---------------------------- Private Functions --------------------------*/
static void InitPinsAndPPS(void);
static void InitTimer2ForPWM(void);
static void InitOCsForPWM(void);

static void StopMotors(void);
static void SetMotor1(int16_t dutySignedPercent); /* RA1/RB9 */
static void SetMotor2(int16_t dutySignedPercent); /* RB3/RB2 */

static uint16_t DutyPercentToOCrs(uint16_t dutyPercent);

static void DoRotate(uint16_t rotateParam);
static void StartRotateTimer(uint16_t rotateParam);
static void DoTranslate(uint16_t translateParam);
static void StartTapeDetect(void);
static void StartBeaconAlignSearch(void);

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitNavigateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
****************************************************************************/
bool InitNavigateService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;
  
  InitPinsAndPPS();
  InitTimer2ForPWM();
  InitOCsForPWM();
  curState = DEBUG;
  orientState = IDLE;
  
  DB_printf("Navigate Service: init done\r\n");

    if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
    PostNavigateService

 Parameters
     EF_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostNavigateService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunNavigateService
****************************************************************************/

ES_Event_t RunNavigateService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  if (ThisEvent.EventType == ES_END_GAME) {
    DB_printf("Stopping Motors\r\n");
    StopMotors();
  }

  switch (curState) {

      case DEBUG:
          //DB_printf("Debug Case for nav service\r\n");
          if (ThisEvent.EventType == ES_TRANSLATE) {
            DB_printf("Do Translate!\r\n");
            DoTranslate(ThisEvent.EventParam);
          }
          if (ThisEvent.EventType == ES_BEACON_SIGNAL) {
            PostBeaconService(ThisEvent);
            DB_printf("Posting to beacon service!\r\n");
          }
          if (ThisEvent.EventType == ES_BEACON_FOUND) {
              DB_printf("Nav service received beacon signal from beacon service!\r\n");
          }
        break;
      
      case INIT_ORIENT:
          /* this is the initial orientation process, moved over from the init SM
          that was in our original plan since it seems to make more sense here.
          could potentially make this its own service later down the line to 
          reduce clutter */

          switch (orientState) {
            case IDLE:
              // starts in idle until it is told to begin ultrasonics alignment search
              if (ThisEvent.EventType == ES_ALIGN_ULTRASONICS) {
                // start init orientation process
                orientState = ULTRASONIC_ALIGN;
              }
              break;

            case ULTRASONIC_ALIGN {
              // turning until both ultrasonics are active
              StartUltrasonicAlignSearch();
              if (ThisEvent.EventType == ES_BOTH_ULTRASONIC_ACTIVE) {
                DB_printf("Both ultrasonics found! Starting beacon seach now\r\n");
                StopMotors();
                orientState = BEACON_ALIGN;
              }
              break;
            }

            case BEACON_ALIGN {
              // determine which side of the board bot is on
              StartBeaconAlignSearch(void); // turn CC slowly
              if (ThisEvent.EventType == ES_BEACON_DETECTED) {
                uint8_t side = ThisEvent.EventParam;

                // post to follower spi service so it can tell leader that side has been found
                if (side == LEFT || side == RIGHT) {
                  StopMotors();
                  ES_Event_t SideEvent;
                  SideEvent.EventType = ES_SIDE_INDICATED;
                  SideEvent.EventParam = side;
                  PostSPIFollowerService(SideEvent);
                  DB_printf("Side determined! Bot is in %d arena\r\n", side);
                  curState = FIRST_COLLECT;
                } else {
                  DB_printf("Invalid freq detected. Will keep searching\r\n");
                }
              }
            break;
            }
          }

    case FIRST_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {

        ES_Event_t AlignEvent;
        AlignEvent.EventType = ES_ALIGN_COLLECT;
        AlignEvent.EventParam = FIRST;
        PostNavigateService(AlignEvent);

        curState = COLLECT_ALIGN;
      }
      break;


    case COLLECT_ALIGN:

      if (ThisEvent.EventType == ES_ALIGN_COLLECT) {
        //StartCollectSM(ThisEvent.EventParam);
      }

      if (ThisEvent.EventType == ES_FIND_BUCKET) {

        if (ThisEvent.EventParam == FIRST) {
          curState = FIRST_DISPENSE;
        }
        else if (ThisEvent.EventParam == MIDDLE) {
          curState = INIT_FIND_MIDDLE;
        }
        else if (ThisEvent.EventParam == END) {
          curState = INIT_FIND_END;
        }
      }
      break;


    case FIRST_DISPENSE:

      if (ThisEvent.EventType == ES_DISPENSE) {
        //StartDispenseSM(FULL);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_FIND_MIDDLE;
      }
      break;


    case INIT_FIND_MIDDLE:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = ALIGN_MID_BUCKET;
      }
      break;


    case ALIGN_MID_BUCKET:

      if (ThisEvent.EventType == ES_DISPENSE) {
        //StartDispenseSM(SPLIT1);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_FIND_END;
      }
      break;


    case INIT_FIND_END:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = ALIGN_END_BUCKET;
      }
      break;


    case ALIGN_END_BUCKET:

      if (ThisEvent.EventType == ES_DISPENSE) {
        //StartDispenseSM(SPLIT2);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_REPEAT_COLLECT;
      }
      break;


    case INIT_REPEAT_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = FIND_COLLECT;
      }
      break;


    case FIND_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {

        ES_Event_t AlignEvent;
        AlignEvent.EventType = ES_ALIGN_COLLECT;
        //AlignEvent.EventParam = OTHER;
        //PostNavigateService(AlignEvent);

        curState = COLLECT_ALIGN;
      }
      break;
  }

  return ReturnEvent;
}

/*=========================== INIT HELPERS ============================*/
static void InitPinsAndPPS(void)
{
  /* motor pins as digital outputs */
  TRISAbits.TRISA4 = 0;  ANSELAbits.ANSA4 = 0; // left motor direction, OC4
  TRISBbits.TRISB5 = 0;  ANSELBbits.ANSB5 = 0; // right motor PWM, OC2
  TRISBbits.TRISB7 = 0;  ANSELBbits.ANSB7 = 0; // left motor PWM, OC1
  TRISBbits.TRISB9 = 0;  ANSELBbits.ANSB9 = 0; // right motor direction, OC3
  
  TRISBbits.TRISB4 = 0; LATBbits.LATB4 = 0; // LED align
  
  #define PPS_FN_PWM 0b0101
  RPA4Rbits.RPA4R = PPS_FN_PWM;
  RPB5Rbits.RPB5R = PPS_FN_PWM;
  RPB7Rbits.RPB7R = PPS_FN_PWM;
  RPB9Rbits.RPB9R = PPS_FN_PWM;
}

static void InitTimer2ForPWM(void)
{
  T2CON = 0;
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = T2_PRESCALE_BITS;
  TMR2 = 0;
  PR2 = (uint16_t)PR2_VALUE;
  T2CONbits.ON = 1;
}

static void InitOCsForPWM(void)
{
  /* all OCs use Timer2 (OCTSEL=0) and PWM mode (OCM=0b110) */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC2CON = 0; OC2R = 0; OC2RS = 0; OC2CONbits.OCTSEL = 0; OC2CONbits.OCM = 0b110; OC2CONbits.ON = 1;
  OC3CON = 0; OC3R = 0; OC3RS = 0; OC3CONbits.OCTSEL = 0; OC3CONbits.OCM = 0b110; OC3CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;
}

/*=========================== MOTOR OUTPUT ============================*/
static uint16_t DutyPercentToOCrs(uint16_t dutyPercent)
{
  if (dutyPercent > 100u) dutyPercent = 100u;
  return (uint16_t)(((uint32_t)dutyPercent * (uint32_t)(PR2 + 1u)) / 100u);
}

static void StopMotors(void)
{
  OC2RS = 0; /* Motor1 IN1, RB5 */
  OC3RS = 0; /* Motor1 IN2, RB9 */
  OC1RS = 0; /* Motor2 IN3, RB7 */
  OC4RS = 0; /* Motor2 IN4, RA4 */
}

/* Motor1 uses RB5(OC2)=IN1 and RB9(OC3)=IN2 */
static void SetMotor1(int16_t dutySignedPercent) // left
{
    DB_printf("Motor1 set %d\n", dutySignedPercent);
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC2RS = ocrs*PR2/100;
    OC3RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC2RS = 0;
    OC3RS = ocrs*PR2/100;
  }
  else
  {
    OC2RS = 0;
    OC3RS = 0;
  }
}

/* Motor2 uses RB7(OC1)=IN3 and RA4(OC4)=IN4 */
static void SetMotor2(int16_t dutySignedPercent) // right
{
  DB_printf("Motor2 set, %d\n", dutySignedPercent);
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC1RS = ocrs*PR2/100;
    OC4RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC1RS = 0;
    OC4RS = ocrs*PR2/100;
  }
  else
  {
    OC1RS = 0;
    OC4RS = 0;
  }
}

/*=========================== MOTION PRIMITIVES ============================*/

static void DoTranslate(uint16_t translateParam)
{
  TransSpeed_t spd;
  TransDir_t dir;
  UnpackTranslateParam(translateParam, &spd, &dir);
  uint16_t duty = (spd == TRANS_FULL) ? DUTY_TRANS_FULL : DUTY_TRANS_HALF;
  int16_t signedDuty = (dir == DIR_FWD) ? (int16_t)duty : -(int16_t)duty;
  SetMotor1(signedDuty);
  SetMotor2(signedDuty);
}

static void DoRotate(uint16_t rotateParam)
{
  RotAngle_t ang;
  RotDir_t dir;
  UnpackRotateParam(rotateParam, &ang, &dir);
  (void)ang;

  if (dir == ROT_CW)
  {
    SetMotor2((int16_t)DUTY_ROTATE);
    SetMotor1(-(int16_t)DUTY_ROTATE);
  }
  else
  {
    SetMotor2(-(int16_t)DUTY_ROTATE);
    SetMotor1((int16_t)DUTY_ROTATE);
  }
}

static void StartRotateTimer(uint16_t rotateParam)
{
  RotAngle_t ang;
  RotDir_t dir;
  UnpackRotateParam(rotateParam, &ang, &dir);
  (void)dir;

  if (ang == ROT_45)
  {
    ES_Timer_InitTimer(ROTATE_TIMER, ROTATE_45_TIME_MS);
  }
  else
  {
    ES_Timer_InitTimer(ROTATE_TIMER, ROTATE_90_TIME_MS);
  }
}

static void StartTapeDetect(void)
{
  /* just move forward until ReflectiveSense posts ES_TAPE_FOUND */
  SetMotor1((int16_t)DUTY_TRANS_HALF);
  SetMotor2((int16_t)DUTY_TRANS_HALF);
}

static void StartBeaconAlignSearch(void)
{
  /* slow rotate until BeaconService posts ES_BEACON_FOUND */
  SetMotor1((int16_t)DUTY_SEARCH);
  SetMotor2(-(int16_t)DUTY_SEARCH);
  //DB_printf("rotating");
}

static void StartUltrasonicAlignSearch(void)
{
  /* slow rotate until both ultrasonics read as active */
  SetMotor1((int16_t)DUTY_SEARCH);
  SetMotor2(-(int16_t)DUTY_SEARCH);
  //DB_printf("rotating");
}