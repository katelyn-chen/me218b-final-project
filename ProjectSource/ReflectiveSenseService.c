/****************************************************************************
  Module
    ReflectiveSenseService.c  (Lab 8)

  Summary
    - Configure ADC for reflective sensor
    - Periodically sample sensor voltage
    - Apply threshold + confidence logic
    - Post ES_TAPE_FOUND when black tape is detected
 * 
 * AW - The line sensor is a digital  output not analog so need to change this

****************************************************************************/

#include "ReflectiveSenseService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

//#include "Lab8_Events.h"
#include "MotorService.h"

// dont need
//#include "PIC32_AD_Lib.h"

/*============================== CONFIG ==============================*/
/* pick an ES timer ID that is free in your ES_Configure.c */
#ifndef REFLECT_TIMER
#define REFLECT_TIMER      3u
#endif

#define REFLECT_SAMPLE_MS  10u

///* Choose which analog pin your Pololu sensor output is wired to */
////#define REFLECT_AD_PIN     AD_PORTAW  // pin 9 or RA2
//#define REFLECT_AD_PIN     2  // changing to different syntax - need to enable this pin somewhere still
///* threshold + confidence */
//#define TAPE_THRESHOLD     650u   /* adjust after looking at ADC values */
//#define TAPE_CONFIRM_N     4u     /* must see tape N samples in a row */
// adding to config line sensor - AW
#define REFLECT_PORT   PORTAbits.RA2
#define REFLECT_TRIS   TRISAbits.TRISA2
#define REFLECT_ACTIVE_HIGH   0
#define TAPE_CONFIRM_N        4u

/*============================== STATE ==============================*/
static uint8_t MyPriority;
static uint8_t TapeCount = 0;
static bool TapeLatched = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitReflectiveHardware(void);
static uint16_t ReadReflectDigital(void);

/*=========================== PUBLIC API =============================*/
bool InitReflectiveSenseService(uint8_t Priority)
{
  MyPriority = Priority;

  InitReflectiveHardware();

  ES_Timer_InitTimer(REFLECT_TIMER, REFLECT_SAMPLE_MS);

  DB_printf("ReflectiveSenseService: init done\r\n");

  ES_Event_t ThisEvent = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostReflectiveSenseService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunReflectiveSenseService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == REFLECT_TIMER))
  {
      // start here
      bool tapeDetected = ReadReflectDigital();

      if (tapeDetected)
      {
        TapeCount++;
        //DB_printf("Y");
      }
      else
      {
        TapeCount = 0;
        //DB_printf("N");
        TapeLatched = false;
      }

      if (!TapeLatched && (TapeCount >= TAPE_CONFIRM_N))
      {
        TapeLatched = true;
        ES_Event_t e = { ES_TAPE_FOUND, 1 };
        PostMotorService(e);
    }

    ES_Timer_InitTimer(REFLECT_TIMER, REFLECT_SAMPLE_MS);
  }

  return ReturnEvent;
}

static void InitReflectiveHardware(void)
{
  /* Typical ME218 ADC init */
 // AD_Init(1);
 // AD_AddPins(REFLECT_AD_PIN);
    REFLECT_TRIS = 1; // make an input
}

static uint16_t ReadReflectDigital(void)
{
#if REFLECT_ACTIVE_HIGH
  return (REFLECT_PORT != 0);
#else
  return (REFLECT_PORT == 0);
#endif
}
