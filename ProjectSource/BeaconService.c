/****************************************************************************
  Module
    BeaconService.c  (Lab 8)

  Summary
    - Configure a digital input + timing hardware (Input Capture)
    - Measure period of IR beacon pulses
    - Convert to frequency
    - If frequency is ~1427 Hz consistently -> post ES_BEACON_FOUND

  Notes
    - using Timer3 + IC2 in this version.
    - MUST set the IC2 input mapping to match the pin we wired
      the beacon sensor output to.
****************************************************************************/

#include "BeaconService.h"

#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "MotorService.h"

/*============================== CONFIG ==============================*/
#define PBCLK_HZ              20000000u

/* Timer3 prescale for capture timing */
#define T3_PRESCALE_BITS      0b010   /* 1:4 (you can change) */
#define T3_PRESCALE_VAL       4u

/* expected beacon frequency */
#define BEACON_FREQ_HZ        1427u

/* tolerance (Hz) */
#define BEACON_TOL_HZ         1000u

/* require this many “good” cycles in a row before declaring found */
#define BEACON_CONFIRM_COUNT  6u

/* which ES timer to use if you want periodic debug (optional) */
#ifndef BEACON_DEBUG_TIMER
#define BEACON_DEBUG_TIMER    2u
#endif
#define BEACON_DEBUG_MS       200u

/* IC2 input select:
   You MUST set this to match your actual pin mapping.
   Example: IC2R=0b0011 might map to RB10 on some configs, but this is board-dependent.
*/
#define IC2R_VALUE            0b0011 //changed from 0b0011 to 0b0000

/*============================== STATE ==============================*/
static uint8_t MyPriority;

static volatile uint32_t LastCapture = 0;
static volatile uint32_t PeriodTicks = 0;
static volatile bool     NewPeriod = false;
static volatile uint32_t RolloverCount = 0;

static uint8_t GoodCount = 0;
static bool BeaconLatched = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitBeaconHardware(void);
static uint32_t ComputeFreqHz(uint32_t periodTicks);

/*=========================== PUBLIC API =============================*/
bool InitBeaconService(uint8_t Priority)
{
  MyPriority = Priority;

  InitBeaconHardware();

  /* optional debug timer */
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

  /* main logic: if we got a new period measurement, check frequency */
  if (ThisEvent.EventType == ES_BEACON_SIGNAL) {
    DB_printf("Beacon case\n");
    if (NewPeriod)
    {
      NewPeriod = false;
      DB_printf("run beacon\n");
      if (PeriodTicks != 0)
      {
        uint32_t f = ComputeFreqHz(PeriodTicks);
        DB_printf("IR frequency%d\n", f);

        /* simple tolerance check */
        if ((f > (BEACON_FREQ_HZ - BEACON_TOL_HZ)) && (f < (BEACON_FREQ_HZ + BEACON_TOL_HZ)))
        {
          if (GoodCount < 255) GoodCount++;
        }
        else
        {
          GoodCount = 0;
          BeaconLatched = false;
        }

        /* only post once per “found” */
        if (!BeaconLatched && (GoodCount >= BEACON_CONFIRM_COUNT))
        {
          DB_printf("Beacon latched!!!!! Good Count: %d", GoodCount);
          BeaconLatched = true;

          ES_Event_t e = { ES_BEACON_FOUND, 0 };
          PostMotorService(e);
          BeaconLatched=false;
        }
      }
    }
  }

  /* optional periodic debug print */
  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == BEACON_DEBUG_TIMER))
  {
    if (PeriodTicks != 0)
    {
      uint32_t f = ComputeFreqHz(PeriodTicks);
      DB_printf("Beacon f=%du Hz good=%u latched=%u\r\n", f, GoodCount, BeaconLatched);
    }
    ES_Timer_InitTimer(BEACON_DEBUG_TIMER, BEACON_DEBUG_MS);
  }

  return ReturnEvent;
}

/*=========================== HARDWARE INIT ============================*/
static void InitBeaconHardware(void)
{
  TRISBbits.TRISB10 = 1;   // input

  /* Timer3 timebase */
  T3CON = 0;
  T3CONbits.TCS = 0;
  T3CONbits.TCKPS = T3_PRESCALE_BITS;
  TMR3 = 0;
  PR3 = 0xFFFF;
  /* Timer3 overflow interrupt to track rollovers (optional but helps) */
  IPC3bits.T3IP = 5;
  IFS0CLR = _IFS0_T3IF_MASK;  
  IEC0SET = _IEC0_T3IE_MASK;
  
  /* enable multi-vector interrupts */
  INTCONbits.MVEC = 1;
 
  IC2CON = 0;

  /* IC2 mapping */
  IC2R = IC2R_VALUE;
  /* IC2 setup: capture rising edges using Timer3 */
  IC2CONbits.C32 = 0;
  IC2CONbits.ICTMR = 0;       /* 0 -> Timer3 */
  IC2CONbits.ICM = 0b011;     /* every rising edge */
  IC2CONbits.ICI = 0b000;

  IPC2bits.IC2IP = 6;
  IEC0SET = _IEC0_IC2IE_MASK;
  IFS0CLR = _IFS0_IC2IF_MASK;
  
  T3CONbits.ON = 1;
  IC2CONbits.ON = 1;

  __builtin_enable_interrupts();
}

static uint32_t ComputeFreqHz(uint32_t periodTicks)
{
  /* ticks -> seconds: tickRate = PBCLK / prescale */
  uint32_t tickRate = (PBCLK_HZ / T3_PRESCALE_VAL);
  if (periodTicks == 0) return 0;
  return (tickRate / periodTicks);
}

/*============================== ISRs ==============================*/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL6SOFT) IC2Handler(void)
{
//  DB_printf("Input Capture");
  uint32_t cap = IC2BUF; /* reading clears the buffer entry */
  IFS0CLR = _IFS0_IC2IF_MASK;
    if(IFS0bits.T3IF && (cap < 0x8000)) { 
//         RolloverCount++;
         IFS0CLR = _IFS0_T3IF_MASK;
     }
  uint32_t current = (RolloverCount << 16) | cap;
  PeriodTicks = current - LastCapture;
  LastCapture = current;
  ES_Event_t ISR;
  ISR.EventType = ES_BEACON_SIGNAL;
  NewPeriod = true;
  PostBeaconService(ISR);
}

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) T3Handler(void)
{
  __builtin_disable_interrupts();
  if (IFS0bits.T3IF) {
         RolloverCount++;
         IFS0CLR = _IFS0_T3IF_MASK;
   }
  __builtin_enable_interrupts();

}
