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
#include "BeaconService.h"

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

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  DEBUG,
  INIT_ORIENT,
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
  ORIENT_IDLE,
  ORIENT_BEACON_SWEEP,
  ORIENT_DONE
} InitOrientState_t;

/*
  Field identification:
    FIELD_GREEN means beacons appear CCW in the BLRG cycle (with rotation offset).
    FIELD_BLUE  means beacons appear CCW in the GRLB cycle (with rotation offset).
*/
typedef enum {
  FIELD_UNKNOWN = 0,
  FIELD_GREEN   = 1,
  FIELD_BLUE    = 2
} Field_t;

/*---------------------------- Module Variables --------------------------*/
static uint8_t           MyPriority;
static NavigateState_t   curState;
static InitOrientState_t orientState;

/* beacon sweep history (store unique transitions only) */
#define BEACON_SEQ_MAX 8u
static BeaconId_t Seq[BEACON_SEQ_MAX];
static uint8_t SeqLen = 0u;

/*---------------------------- Private Functions --------------------------*/
static void InitPinsAndPPS(void);
static void InitTimer2ForPWM(void);
static void InitOCsForPWM(void);

static void StopMotors(void);
static void SetMotor1(int16_t dutySignedPercent); /* left motor */
static void SetMotor2(int16_t dutySignedPercent); /* right motor */

static uint16_t DutyPercentToOCrs(uint16_t dutyPercent);

static void DoRotate(uint16_t rotateParam);
static void DoTranslate(uint16_t translateParam);
static void StartTapeDetect(void);
static void StartBeaconAlignSearch(void);

static void ResetBeaconSequence(void);
static bool PushBeaconIfNew(BeaconId_t id);
static Field_t DetermineFieldFromSequence(void);

/*------------------------------ Module Code ------------------------------*/

bool InitNavigateService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;

  InitPinsAndPPS();
  InitTimer2ForPWM();
  InitOCsForPWM();

  curState = DEBUG;
  orientState = ORIENT_IDLE;
  ResetBeaconSequence();

  DB_printf("Navigate Service: init done\r\n");

  ThisEvent.EventType  = ES_INIT;
  ThisEvent.EventParam = 0;

  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostNavigateService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunNavigateService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType  = ES_NO_EVENT;
  ReturnEvent.EventParam = 0;

  if (ThisEvent.EventType == ES_END_GAME)
  {
    DB_printf("Stopping Motors\r\n");
    StopMotors();
    return ReturnEvent;
  }

  switch (curState)
  {
    case DEBUG:
    {
      if (ThisEvent.EventType == ES_TRANSLATE)
      {
        DB_printf("Do Translate!\r\n");
        DoTranslate(ThisEvent.EventParam);
      }
      else if (ThisEvent.EventType == ES_ROTATE)
      {
        DB_printf("Do Rotate!\r\n");
        DoRotate(ThisEvent.EventParam);
      }
      else if (ThisEvent.EventType == ES_TAPE_DETECT)
      {
        DB_printf("Start Tape Detect\r\n");
        StartTapeDetect();
      }
      else if (ThisEvent.EventType == ES_ALIGN_ULTRASONICS)
      {
        /*
          Reusing ES_ALIGN_ULTRASONICS as the “begin initial orientation” trigger.
          Initial orientation is now beacon-only: rotate CCW and classify field
          based on the beacon order.
        */
        DB_printf("Starting INIT_ORIENT (beacon sweep)\r\n");
        curState = INIT_ORIENT;
        orientState = ORIENT_BEACON_SWEEP;

        ResetBeaconSequence();
        StartBeaconAlignSearch();
      }
      break;
    }

    case INIT_ORIENT:
    {
      switch (orientState)
      {
        case ORIENT_IDLE:
        {
          if (ThisEvent.EventType == ES_ALIGN_ULTRASONICS)
          {
            orientState = ORIENT_BEACON_SWEEP;
            ResetBeaconSequence();
            StartBeaconAlignSearch();
          }
          break;
        }

        case ORIENT_BEACON_SWEEP:
        {
          if (ThisEvent.EventType == ES_BEACON_FOUND)
          {
            BeaconId_t id;
            BeaconSide_t side;
            UnpackBeaconParam((uint16_t)ThisEvent.EventParam, &id, &side);
            (void)side;

            if (id != BEACON_ID_NONE)
            {
              bool pushed = PushBeaconIfNew(id);
              if (pushed)
              {
                DB_printf("Beacon seq add: id=%u (len=%u)\r\n",
                          (unsigned)id, (unsigned)SeqLen);

                Field_t field = DetermineFieldFromSequence();
                if (field != FIELD_UNKNOWN)
                {
                  StopMotors();

                  ES_Event_t SideEvent;
                  SideEvent.EventType = ES_SIDE_INDICATED;
                  SideEvent.EventParam = (field == FIELD_GREEN) ? 0u : 1u; /* 0=GREEN, 1=BLUE */

                  PostSPIFollowerService(SideEvent);

                  DB_printf("Field determined: %u\r\n", (unsigned)field);

                  orientState = ORIENT_DONE;
                  curState = FIRST_COLLECT;
                }
              }
            }
          }
          break;
        }

        case ORIENT_DONE:
        default:
          break;
      }
      break;
    }

    case FIRST_COLLECT:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        ES_Event_t AlignEvent;
        AlignEvent.EventType  = ES_ALIGN_COLLECT;
        AlignEvent.EventParam = FIRST;
        PostNavigateService(AlignEvent);

        curState = COLLECT_ALIGN;
      }
      break;
    }

    case COLLECT_ALIGN:
    {
      if (ThisEvent.EventType == ES_FIND_BUCKET)
      {
        if (ThisEvent.EventParam == FIRST)       curState = FIRST_DISPENSE;
        else if (ThisEvent.EventParam == MIDDLE) curState = INIT_FIND_MIDDLE;
        else if (ThisEvent.EventParam == END)    curState = INIT_FIND_END;
      }
      break;
    }

    case FIRST_DISPENSE:
    {
      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
      {
        curState = INIT_FIND_MIDDLE;
      }
      break;
    }

    case INIT_FIND_MIDDLE:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = ALIGN_MID_BUCKET;
      }
      break;
    }

    case ALIGN_MID_BUCKET:
    {
      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
      {
        curState = INIT_FIND_END;
      }
      break;
    }

    case INIT_FIND_END:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = ALIGN_END_BUCKET;
      }
      break;
    }

    case ALIGN_END_BUCKET:
    {
      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
      {
        curState = INIT_REPEAT_COLLECT;
      }
      break;
    }

    case INIT_REPEAT_COLLECT:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = FIND_COLLECT;
      }
      break;
    }

    case FIND_COLLECT:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = COLLECT_ALIGN;
      }
      break;
    }

    case WAITING_FOR_SIDE:
    default:
      break;
  }

  return ReturnEvent;
}

/*=========================== INIT HELPERS ============================*/
static void InitPinsAndPPS(void)
{
  /* motor pins as digital outputs (update these mappings if motor wiring changes) */
  TRISAbits.TRISA1 = 0;  ANSELAbits.ANSA1 = 0;
  TRISBbits.TRISB3 = 0;  ANSELBbits.ANSB3 = 0;
  TRISBbits.TRISB2 = 0;  ANSELBbits.ANSB2 = 0;

  TRISBbits.TRISB4 = 0; LATBbits.LATB4 = 0; /* LED align */

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
}

/*=========================== MOTOR OUTPUT ============================*/
static uint16_t DutyPercentToOCrs(uint16_t dutyPercent)
{
  if (dutyPercent > 100u) dutyPercent = 100u;
  return (uint16_t)(((uint32_t)dutyPercent * (uint32_t)(PR2 + 1u)) / 100u);
}

static void StopMotors(void)
{
  OC2RS = 0;
  OC3RS = 0;
  OC1RS = 0;
  OC4RS = 0;
}

static void SetMotor1(int16_t dutySignedPercent)
{
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

static void SetMotor2(int16_t dutySignedPercent)
{
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

static void StartTapeDetect(void)
{
  SetMotor1((int16_t)DUTY_TRANS_HALF);
  SetMotor2((int16_t)DUTY_TRANS_HALF);
}

static void StartBeaconAlignSearch(void)
{
  /* CCW in-place sweep to collect beacon ordering */
  SetMotor1((int16_t)DUTY_SEARCH);
  SetMotor2(-(int16_t)DUTY_SEARCH);
}

/*=========================== BEACON ORDER LOGIC ============================*/
static void ResetBeaconSequence(void)
{
  for (uint8_t i = 0u; i < BEACON_SEQ_MAX; i++) Seq[i] = BEACON_ID_NONE;
  SeqLen = 0u;
}

static bool PushBeaconIfNew(BeaconId_t id)
{
  if (SeqLen > 0u)
  {
    if (Seq[SeqLen - 1u] == id) return false;
  }

  if (SeqLen < BEACON_SEQ_MAX)
  {
    Seq[SeqLen++] = id;
  }
  else
  {
    /* shift left and append */
    for (uint8_t i = 1u; i < BEACON_SEQ_MAX; i++) Seq[i - 1u] = Seq[i];
    Seq[BEACON_SEQ_MAX - 1u] = id;
  }
  return true;
}

static bool MatchCycle(const BeaconId_t *cycle4, const BeaconId_t *obs, uint8_t nObs)
{
  /* allow any rotation offset of the 4-cycle */
  for (uint8_t offset = 0u; offset < 4u; offset++)
  {
    bool ok = true;
    for (uint8_t k = 0u; k < nObs; k++)
    {
      BeaconId_t expect = cycle4[(offset + k) & 0x03u];
      if (obs[k] != expect)
      {
        ok = false;
        break;
      }
    }
    if (ok) return true;
  }
  return false;
}

static Field_t DetermineFieldFromSequence(void)
{
  /*
    Green field CCW cycle: B L R G
    Blue  field CCW cycle: G R L B

    Require 5 unique transitions before deciding to reduce false positives.
  */
  if (SeqLen < 5u) return FIELD_UNKNOWN;

  BeaconId_t last5[5];
  for (uint8_t i = 0u; i < 5u; i++)
  {
    last5[i] = Seq[SeqLen - 5u + i];
  }

  const BeaconId_t greenCycle[4] = { BEACON_ID_B, BEACON_ID_L, BEACON_ID_R, BEACON_ID_G };
  const BeaconId_t blueCycle[4]  = { BEACON_ID_G, BEACON_ID_R, BEACON_ID_L, BEACON_ID_B };

  bool greenOK = MatchCycle(greenCycle, last5, 5u);
  bool blueOK  = MatchCycle(blueCycle,  last5, 5u);

  if (greenOK && !blueOK) return FIELD_GREEN;
  if (blueOK  && !greenOK) return FIELD_BLUE;

  return FIELD_UNKNOWN;
}