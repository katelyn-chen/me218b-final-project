/****************************************************************************
  Module
    ReflectiveSenseService.c  (Lab 8)

  Summary
    - Receives ES_TAPE_CHANGE event from event checker when tape sensor state changes
    - Determine what tape change means
****************************************************************************/

#include "ReflectiveSenseService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "NavigateService.h"
#include "SPIFollowerService.h"

/*============================== CONFIG ==============================*/

// Tape left to right from robot's perspective (1-5): RB4, RB5, RB11, RB12, RB13
// add fuzzy checking for corners? verify with hardware to see accuracy
#define CENTER_BITFIELD           0b00100
#define OFFCENTER_RIGHT_BITFIELD  0b00110 // middle + right
#define OFFCENTER_LEFT_BITFIELD   0b01100 // middle + left
#define ALL_TAPE_BITFIELD         0b11111
#define RIGHT_CORNER_BITFIELD     0b00111
#define LEFT_CORNER_BITFIELD      0b11100

#define TAPE_CONFIRM_COUNT       5

/*============================== STATE ==============================*/
static uint8_t MyPriority;
static uint8_t lastConfirmedState;
static uint8_t debounceCounter;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitReflectiveHardware(void);

/*=========================== PUBLIC API =============================*/
bool InitReflectiveSenseService(uint8_t Priority)
{
  MyPriority = Priority;
  InitReflectiveHardware();
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

  switch (ThisEvent.EventType)
  {    
    case ES_INIT:
    {
        DB_printf("ReflectiveSenseService received ES_INIT\r\n");
        break;
    }
    case ES_TAPE_CHANGE:
    {
        DB_printf("Tape change detected! Sensor bitfield: %d\r\n", ThisEvent.EventParam);
        uint8_t tapeState = (uint8_t)ThisEvent.EventParam;
        static uint8_t pendingState = 0;

        if (tapeState == pendingState) {
            debounceCounter++;
        } else {
            pendingState = tapeState;
            debounceCounter = 1;
        }

        if (debounceCounter >= TAPE_CONFIRM_COUNT && tapeState != lastConfirmedState) {
          lastConfirmedState = tapeState;
          if (tapeState == ALL_TAPE_BITFIELD) { 
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = FULL_T;
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == RIGHT_CORNER_BITFIELD) {
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = RIGHT_CORNER;
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == LEFT_CORNER_BITFIELD) {
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = LEFT_CORNER;
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == CENTER_BITFIELD) { 
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_CENTERED; 
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == OFFCENTER_RIGHT_BITFIELD) {
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_OFF_CENTER_RIGHT; 
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == OFFCENTER_LEFT_BITFIELD) {
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_OFF_CENTER_LEFT; 
              PostSPIFollowerService(NewEvent);
          } else if (tapeState == 0) { // do we need know no tape??
              ES_Event_t NewEvent;
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = NO_TAPE;
              PostSPIFollowerService(NewEvent);
          }
      }
      break;
    }
  }
  return ReturnEvent;
}

static void InitReflectiveHardware(void)
{
    PIN_MapPinInput(TapeSensor1);
    PIN_MapPinInput(TapeSensor2);
    PIN_MapPinInput(TapeSensor3);
    PIN_MapPinInput(TapeSensor4);
    PIN_MapPinInput(TapeSensor5);
}