/****************************************************************************
  Module
    BeaconService.c  (Lab 8)

  Summary
    - Configure a digital input + timing hardware (Input Capture)
    - Measure period of IR beacon pulses
    - Convert to frequency
    - Classify against the four Seesaw beacons
    - If a beacon is consistent -> post ES_BEACON_FOUND with parameter

  Notes
    - using Timer3 + IC1 + RA2
****************************************************************************/

#include "BeaconService.h"

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"
#include "NavigateService.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ              20000000u

/* Timer3 prescale for capture timing */
#define T3_PRESCALE_BITS      0b010u   /* 1:4 */
#define T3_PRESCALE_VAL       4u

/* Seesaw IR beacon frequencies (Hz) */
#define BEACON_G_HZ           3333u
#define BEACON_B_HZ           1427u
#define BEACON_R_HZ            909u
#define BEACON_L_HZ           2000u

/* Per-beacon tolerance (Hz) */
#define BEACON_G_TOL_HZ        250u
#define BEACON_B_TOL_HZ        150u
#define BEACON_R_TOL_HZ        120u
#define BEACON_L_TOL_HZ        150u

/* require this many good cycles in a row before declaring found */
#define BEACON_CONFIRM_COUNT   15u

/* which ES timer to use if periodic debug is desired */
#ifndef BEACON_DEBUG_TIMER
#define BEACON_DEBUG_TIMER     2u
#endif
#define BEACON_DEBUG_MS        200u

/* IC1 input select */
#define IC1R_VALUE             0b0000u /* RA2 -> IC1 */

/*
  Bucket beacons mapping:
*/
#define LEFT_BUCKET_BEACON_ID   BEACON_ID_L   /* 2000 Hz */
#define RIGHT_BUCKET_BEACON_ID  BEACON_ID_R   /*  909 Hz */

/*============================== STATE ==============================*/
static uint8_t MyPriority;

static volatile uint32_t LastCapture   = 0u;
static volatile uint32_t PeriodTicks   = 0u;
static volatile bool     NewPeriod     = false;
static volatile uint32_t RolloverCount = 0u;

static uint8_t   GoodCount     = 0u;
static bool      BeaconLatched = false;
static BeaconId_t Candidate    = BEACON_ID_NONE;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitBeaconHardware(void);
static uint32_t ComputeFreqHz(uint32_t periodTicks);
static BeaconId_t ClassifyBeacon(uint32_t fHz);
static BeaconSide_t ClassifySide(BeaconId_t id);

/*=========================== PUBLIC API =============================*/
bool InitBeaconService(uint8_t Priority)
{
  MyPriority = Priority;

  InitBeaconHardware();

  ES_Timer_InitTimer(BEACON_DEBUG_TIMER, BEACON_DEBUG_MS);

  DB_printf("BeaconService: init done\r\n");

  ES_Event_t ThisEvent = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostBeaconService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunBeaconService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  if (ThisEvent.EventType == ES_BEACON_SIGNAL)
  {
    if (NewPeriod)
    {
      NewPeriod = false;

      if (PeriodTicks != 0u)
      {
        uint32_t f = ComputeFreqHz(PeriodTicks);
        BeaconId_t id = ClassifyBeacon(f);

        if (id != BEACON_ID_NONE)
        {
          /* same candidate repeatedly -> count up, otherwise restart candidate */
          if (id == Candidate)
          {
            if (GoodCount < 255u) GoodCount++;
          }
          else
          {
            Candidate = id;
            GoodCount = 1u;
            BeaconLatched = false;
          }

          /* latch once per candidate; resets automatically when candidate changes */
          if (!BeaconLatched && (GoodCount >= BEACON_CONFIRM_COUNT))
          {
            BeaconLatched = true;

            BeaconSide_t side = ClassifySide(id);
            uint16_t packed = PackBeaconParam(id, side);

            DB_printf("Beacon latched: id=%u f=%lu Hz side=%u\r\n",
                      (unsigned)id, (unsigned long)f, (unsigned)side);

            ES_Event_t e = { ES_BEACON_FOUND, packed };
            PostNavigateService(e);
          }
        }
        else
        {
          Candidate = BEACON_ID_NONE;
          GoodCount = 0u;
          BeaconLatched = false;
        }
      }
    }
  }

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == BEACON_DEBUG_TIMER))
  {
    if (PeriodTicks != 0u)
    {
      uint32_t f = ComputeFreqHz(PeriodTicks);
      (void)f;
      /* DB_printf("Beacon f=%lu Hz cand=%u good=%u latched=%u\r\n",
                 (unsigned long)f, (unsigned)Candidate, (unsigned)GoodCount, (unsigned)BeaconLatched); */
    }
    ES_Timer_InitTimer(BEACON_DEBUG_TIMER, BEACON_DEBUG_MS);
  }

  return ReturnEvent;
}

/*=========================== HARDWARE INIT ============================*/
static void InitBeaconHardware(void)
{
  /* RA2 as digital input for IC1 */
  TRISAbits.TRISA2 = 1;

  /* Timer3 timebase */
  T3CON = 0;
  T3CONbits.TCS = 0;
  T3CONbits.TCKPS = T3_PRESCALE_BITS;
  TMR3 = 0;
  PR3 = 0xFFFF;

  /* Timer3 overflow interrupt to track rollovers */
  IPC3bits.T3IP = 5;
  IFS0CLR = _IFS0_T3IF_MASK;
  IEC0SET = _IEC0_T3IE_MASK;

  /* enable multi-vector interrupts */
  INTCONbits.MVEC = 1;

  IC1CON = 0;

  /* IC1 mapping */
  IC1R = IC1R_VALUE;

  /* IC1 setup: capture rising edges using Timer3 */
  IC1CONbits.C32 = 0;
  IC1CONbits.ICTMR = 0;       /* 0 -> Timer3 */
  IC1CONbits.ICM = 0b011;     /* every rising edge */
  IC1CONbits.ICI = 0b000;

  IPC1bits.IC1IP = 6;
  IEC0SET = _IEC0_IC1IE_MASK;
  IFS0CLR = _IFS0_IC1IF_MASK;

  T3CONbits.ON = 1;
  IC1CONbits.ON = 1;

  __builtin_enable_interrupts();
}

static uint32_t ComputeFreqHz(uint32_t periodTicks)
{
  uint32_t tickRate = (PBCLK_HZ / T3_PRESCALE_VAL);
  if (periodTicks == 0u) return 0u;
  return (tickRate / periodTicks);
}

/*=========================== CLASSIFIER ============================*/
static BeaconId_t ClassifyBeacon(uint32_t fHz)
{
  BeaconId_t best = BEACON_ID_NONE;
  uint32_t bestErr = 0xFFFFFFFFu;

  struct { BeaconId_t id; uint32_t f; uint32_t tol; } candidates[] = {
    { BEACON_ID_G, BEACON_G_HZ, BEACON_G_TOL_HZ },
    { BEACON_ID_B, BEACON_B_HZ, BEACON_B_TOL_HZ },
    { BEACON_ID_R, BEACON_R_HZ, BEACON_R_TOL_HZ },
    { BEACON_ID_L, BEACON_L_HZ, BEACON_L_TOL_HZ }
  };

  for (unsigned i = 0u; i < (sizeof(candidates)/sizeof(candidates[0])); i++)
  {
    uint32_t target = candidates[i].f;
    uint32_t tol    = candidates[i].tol;

    uint32_t err = (fHz > target) ? (fHz - target) : (target - fHz);
    if (err <= tol)
    {
      if (err < bestErr)
      {
        bestErr = err;
        best = candidates[i].id;
      }
    }
  }

  return best;
}

static BeaconSide_t ClassifySide(BeaconId_t id)
{
  if (id == LEFT_BUCKET_BEACON_ID)  return BEACON_SIDE_LEFT;
  if (id == RIGHT_BUCKET_BEACON_ID) return BEACON_SIDE_RIGHT;
  return BEACON_SIDE_UNKNOWN;
}

/*============================== ISRs ==============================*/
void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL6SOFT) IC1Handler(void)
{
  uint32_t cap = IC1BUF;
  IFS0CLR = _IFS0_IC1IF_MASK;

  if (IFS0bits.T3IF && (cap < 0x8000u))
  {
    IFS0CLR = _IFS0_T3IF_MASK;
  }

  uint32_t current = (RolloverCount << 16) | cap;
  PeriodTicks = current - LastCapture;
  LastCapture = current;

  ES_Event_t ISR;
  ISR.EventType  = ES_BEACON_SIGNAL;
  ISR.EventParam = 0;

  NewPeriod = true;
  PostBeaconService(ISR);
}

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) T3Handler(void)
{
  __builtin_disable_interrupts();
  if (IFS0bits.T3IF)
  {
    RolloverCount++;
    IFS0CLR = _IFS0_T3IF_MASK;
  }
  __builtin_enable_interrupts();
}