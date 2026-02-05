/****************************************************************************
  Module
    MotorService.c  (Lab 8)

  Summary
    Central robot behavior controller + motor output manager.
    - Receives motion commands (SPIService)
    - Receives sensor events (BeaconService, ReflectiveSenseService)
    - Drives two motors through an L293 using 4 PWM channels (OC1..OC4)
    - Uses ES timers for time-based actions (fixed-angle rotations)

  Pin / Channel Mapping (wiring from schematic):
    Motor 1:
      RA1 -> IN1 on L293  (OC2)
      RB9 -> IN2 on L293  (OC3)

    Motor 2:
      RB3 -> IN3 on L293  (OC1)
      RB2 -> IN4 on L293  (OC4)

****************************************************************************/

#include "MotorService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

// Commenting out bc moved this out
//#include "Lab8_Events.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ            20000000u
#define PWM_FREQ_HZ         10000u

/* Timer2 @ 10kHz PWM with prescale 1:1 -> PR2 = 1999 */
#define T2_PRESCALE_BITS    (0b000u) /* 1:1 */
#define T2_PRESCALE_VAL     1u
#define PR2_VALUE           ((PBCLK_HZ / (T2_PRESCALE_VAL * PWM_FREQ_HZ)) - 1u)

/* duty presets (percent 0..100) */
#define DUTY_STOP           0u
#define DUTY_TRANS_HALF     35u
#define DUTY_TRANS_FULL     60u
#define DUTY_ROTATE         45u
#define DUTY_SEARCH         30u

/* rotate timing (ms) — will calibrate these on the floor */
// placeholders for now
#define ROTATE_45_TIME_MS   250u
#define ROTATE_90_TIME_MS   500u

/* match with ES timer ID in ES_Configure.h */
#ifndef ROTATE_TIMER
#define ROTATE_TIMER MOTOR_TIMER
#endif


/*============================== STATE ==============================*/
typedef enum {
  MOTOR_STOP = 0,
  MOTOR_ROTATE,
  MOTOR_TRANSLATE,
  MOTOR_TAPE_DETECT,
  MOTOR_BEACON_ALIGN
} MotorState_t;

static uint8_t      MyPriority;
static MotorState_t CurrentState;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
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

/*=========================== PUBLIC API =============================*/
bool InitMotorService(uint8_t Priority)
{
  
    MyPriority = Priority;
    ES_Event_t ThisEvent;
  InitPinsAndPPS();
  InitTimer2ForPWM();
  InitOCsForPWM();

  CurrentState = MOTOR_STOP;
  StopMotors();

  DB_printf("MotorService: init done\r\n");

    if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool PostMotorService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (CurrentState)
  {
    case MOTOR_STOP:
      switch (ThisEvent.EventType)
      {
        case ES_ROTATE:
          DB_printf("rotate run\n");
          DoRotate(ThisEvent.EventParam);
          StartRotateTimer(ThisEvent.EventParam);
          CurrentState = MOTOR_ROTATE;
          break;

        case ES_TRANSLATE:
          DB_printf("translate run\n");
          DoTranslate(ThisEvent.EventParam);
          CurrentState = MOTOR_TRANSLATE;
          break;

        case ES_TAPE_DETECT:
          DB_printf("tape detect\n");
          StartTapeDetect();
          CurrentState = MOTOR_TAPE_DETECT;
          break;

        case ES_ALIGN:
          DB_printf("align\n");
          StartBeaconAlignSearch();
          CurrentState = MOTOR_BEACON_ALIGN;
          break;

        case ES_STOP:
          DB_printf("stop posted!!!\n");
          StopMotors();
          CurrentState = MOTOR_STOP;
          break;
      }
      break;

    case MOTOR_ROTATE:
      DB_printf("motor rotating\n");
      if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == ROTATE_TIMER))
      {
        StopMotors();
        CurrentState = MOTOR_STOP;
      }
      if (ThisEvent.EventType == ES_STOP)
      {
        StopMotors();
        CurrentState = MOTOR_STOP;
      }
      break;

    case MOTOR_TRANSLATE:
      DB_printf("motor translating\n");
      if (ThisEvent.EventType == ES_STOP)
      {
        StopMotors();
        CurrentState = MOTOR_STOP;
      }
      if (ThisEvent.EventType == ES_TRANSLATE)
      {
        DoTranslate(ThisEvent.EventParam);
      }
      break;

    case MOTOR_TAPE_DETECT:
      DB_printf("motor tape detecting\n");
      if (ThisEvent.EventType == ES_TAPE_FOUND)
      {
        StopMotors();
        CurrentState = MOTOR_STOP;
      }
      if (ThisEvent.EventType == ES_STOP)
      {
        StopMotors();
        CurrentState = MOTOR_STOP;
      }
      break;

    case MOTOR_BEACON_ALIGN:
      DB_printf("motor aligning\n");
      LATBbits.LATB4 = 1;
      if (ThisEvent.EventType == ES_BEACON_FOUND)
      {
        StopMotors();
        LATBbits.LATB4 = 0;
        LATBbits.LATB11 = 1;
        CurrentState = MOTOR_STOP;
      }
      if (ThisEvent.EventType == ES_STOP)
      {
        StopMotors();
        LATBbits.LATB4 = 0;
        CurrentState = MOTOR_STOP;
      }
      break;

    default:
      StopMotors();
      CurrentState = MOTOR_STOP;
      break;
  }

  return ReturnEvent;
}

/*=========================== INIT HELPERS ============================*/
static void InitPinsAndPPS(void)
{
  /* motor pins as digital outputs */
  TRISAbits.TRISA1 = 0;  ANSELAbits.ANSA1 = 0;
//  TRISBbits.TRISB9 = 0;  ANSELBbits.ANSB9 = 0;
  TRISBbits.TRISB3 = 0;  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB2 = 0;  ANSELBbits.ANSB2 = 0;
  
  TRISBbits.TRISB4 = 0; LATBbits.LATB4 = 0; // LED align
  TRISBbits.TRISB11 = 0; LATBbits.LATB11 = 0; // LED align done
  
  TRISBbits.TRISB12 = 1; ANSELBbits.ANSB12 = 0; // button

  /* keep PPS exactly like my working setup */
  #define PPS_FN_PWM 0b0101
  RPA1Rbits.RPA1R = PPS_FN_PWM;
  RPB9Rbits.RPB9R = PPS_FN_PWM;
  RPB3Rbits.RPB3R = PPS_FN_PWM;
  RPB2Rbits.RPB2R = PPS_FN_PWM;
}

static void InitTimer2ForPWM(void)
{
  T2CON = 0;
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = T2_PRESCALE_BITS;
  TMR2 = 0;
  PR2 = (uint16_t)PR2_VALUE;
//  IPC2bits.T2IP = 4;
  T2CONbits.ON = 1;
}

static void InitOCsForPWM(void)
{
  /* all OCs use Timer2 (OCTSEL=0) and PWM mode (OCM=0b110) */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC2CON = 0; OC2R = 0; OC2RS = 0; OC2CONbits.OCTSEL = 0; OC2CONbits.OCM = 0b110; OC2CONbits.ON = 1;
  OC3CON = 0; OC3R = 0; OC3RS = 0; OC3CONbits.OCTSEL = 0; OC3CONbits.OCM = 0b110; OC3CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;

  StopMotors();
}

/*=========================== MOTOR OUTPUT ============================*/
static uint16_t DutyPercentToOCrs(uint16_t dutyPercent)
{
  if (dutyPercent > 100u) dutyPercent = 100u;
  return (uint16_t)(((uint32_t)dutyPercent * (uint32_t)(PR2 + 1u)) / 100u);
}

static void StopMotors(void)
{
  OC2RS = 0; /* Motor1 IN1 */
  OC3RS = 0; /* Motor1 IN2 */
  OC1RS = 0; /* Motor2 IN3 */
  OC4RS = 0; /* Motor2 IN4 */
}

/* Motor1 uses RA1(OC2)=IN1 and RB9(OC3)=IN2 */
static void SetMotor1(int16_t dutySignedPercent) // left
{
    DB_printf("Motor1 set\n");
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  
  if (dutySignedPercent > 0)
  {
    OC2RS = 2*ocrs*PR2/100;
    OC3RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC2RS = 0;
    OC3RS = 2*ocrs*PR2/100;
  }
  else
  {
    OC2RS = 0;
    OC3RS = 0;
  }
}

/* Motor2 uses RB3(OC1)=IN3 and RB2(OC4)=IN4 */
static void SetMotor2(int16_t dutySignedPercent) // right
{
  DB_printf("Motor2 set\n");
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC1RS = 2*ocrs*PR2/100;
    OC4RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC1RS = 0;
    OC4RS = 2*ocrs*PR2/100;
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
  DB_printf(signedDuty);
}

static void DoRotate(uint16_t rotateParam)
{
  RotAngle_t ang;
  RotDir_t dir;
  UnpackRotateParam(rotateParam, &ang, &dir);
  (void)ang;

  if (dir == ROT_CW)
  {
    SetMotor1((int16_t)DUTY_ROTATE);
    SetMotor2(-(int16_t)DUTY_ROTATE);
  }
  else
  {
    SetMotor1(-(int16_t)DUTY_ROTATE);
    SetMotor2((int16_t)DUTY_ROTATE);
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
