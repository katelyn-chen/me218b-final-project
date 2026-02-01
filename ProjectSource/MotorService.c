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
#include "Lab8_Events.h"

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

/* rotate timing (ms) — I calibrate these on the floor */
#define ROTATE_45_TIME_MS   250u
#define ROTATE_90_TIME_MS   500u

/* pick an ES timer ID that is free in your ES_Configure.c */
#ifndef ROTATE_TIMER
#define ROTATE_TIMER        0u
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

  InitPinsAndPPS();
  InitTimer2ForPWM();
  InitOCsForPWM();

  CurrentState = MOTOR_STOP;
  StopMotors();

  dbprintf("MotorService: init done\r\n");

  ES_Event_t ThisEvent = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, ThisEvent);
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
          DoRotate(ThisEvent.EventParam);
          StartRotateTimer(ThisEvent.EventParam);
          CurrentState = MOTOR_ROTATE;
          break;

        case ES_TRANSLATE:
          DoTranslate(ThisEvent.EventParam);
          CurrentState = MOTOR_TRANSLATE;
          break;

        case ES_TAPE_DETECT:
          StartTapeDetect();
          CurrentState = MOTOR_TAPE_DETECT;
          break;

        case ES_ALIGN:
          StartBeaconAlignSearch();
          CurrentState = MOTOR_BEACON_ALIGN;
          break;

        case ES_STOP:
        default:
          StopMotors();
          CurrentState = MOTOR_STOP;
          break;
      }
      break;

    case MOTOR_ROTATE:
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
      if (ThisEvent.EventType == ES_BEACON_FOUND)
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
  TRISBbits.TRISB9 = 0;  ANSELBbits.ANSB9 = 0;
  TRISBbits.TRISB3 = 0;  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB2 = 0;  ANSELBbits.ANSB2 = 0;

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
static void SetMotor1(int16_t dutySignedPercent)
{
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC2RS = ocrs;
    OC3RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC2RS = 0;
    OC3RS = ocrs;
  }
  else
  {
    OC2RS = 0;
    OC3RS = 0;
  }
}

/* Motor2 uses RB3(OC1)=IN3 and RB2(OC4)=IN4 */
static void SetMotor2(int16_t dutySignedPercent)
{
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC1RS = ocrs;
    OC4RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC1RS = 0;
    OC4RS = ocrs;
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
}
