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
#include "NavigateService.h"
#include "PIC32_SPI_HAL.h"
/*============================== CONFIG ==============================*/

// Tape left to right from robot's perspective (1-5): RB4, RB5, RB11, RB12, RB13
// add fuzzy checking for corners? verify with hardware to see accuracy
#define CENTER_BITFIELD           0b00100
#define OFFCENTER_RIGHT_BITFIELD  0b00110 // middle + right
#define OFFCENTER_LEFT_BITFIELD   0b01100 // middle + left
#define ALL_TAPE_BITFIELD         0b11111
#define RIGHT_CORNER_BITFIELD     0b00111
#define RIGHT_CORNER_BITFIELD_4   0b01111
#define LEFT_CORNER_BITFIELD      0b11100
#define LEFT_CORNER_BITFIELD_4    0b11110
#define LEFT_BIT_ONLY             0b10000
#define RIGHT_BIT_ONLY            0b00001
#define LEFT_TWO_BITS             0b11000
#define RIGHT_TWO_BITS            0b00011


#define TAPE_CONFIRM_COUNT        5
#define NEXT_T_WAIT               1000        // 1.5 second

/*============================== STATE ==============================*/
static uint8_t MyPriority;
static uint8_t lastConfirmedState;
static uint8_t debounceCounter;
static uint8_t T_count = 0;

/*====================== PRIVATE FUNCTION PROTOS =====================*/

/*=========================== PUBLIC API =============================*/
bool InitReflectiveSenseService(uint8_t Priority)
{
  MyPriority = Priority;
  DB_printf("ReflectiveSenseService: init done\r\n");
  ES_Event_t ThisEvent = { ES_INIT, 0 };
  ES_Timer_StopTimer(LINE_TIMER);
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
//        DB_printf("Tape change detected! Sensor bitfield: %d\r\n", ThisEvent.EventParam);
        uint8_t tapeState = (uint8_t)ThisEvent.EventParam;
          ES_Event_t NewEvent;
          if (tapeState == ALL_TAPE_BITFIELD && T_count == 0 
                  || tapeState == RIGHT_CORNER_BITFIELD_4 || tapeState == LEFT_CORNER_BITFIELD_4) { 
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = FULL_T;
              DB_printf("all tape detected, full T \n");
              PostNavigateService(NewEvent);
              T_count++;
              ES_Timer_InitTimer(LINE_TIMER, NEXT_T_WAIT);
          } else if ((tapeState == RIGHT_CORNER_BITFIELD) && T_count == 0) {
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = RIGHT_CORNER;
              DB_printf("right corner bitfield detected \n");
              PostNavigateService(NewEvent);
              T_count++;
              ES_Timer_InitTimer(LINE_TIMER, NEXT_T_WAIT);
          } else if ((tapeState == LEFT_CORNER_BITFIELD ) && T_count == 0) {
              NewEvent.EventType = ES_T_DETECTED;
              NewEvent.EventParam = LEFT_CORNER;
              DB_printf("left corner bitfield detected \n");
              PostNavigateService(NewEvent);
              T_count++;
              ES_Timer_InitTimer(LINE_TIMER, NEXT_T_WAIT);
          } else if (tapeState == CENTER_BITFIELD) { 
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_CENTERED; 
              DB_printf("center tape detect \n");
              PostNavigateService(NewEvent);
          } else if (tapeState == OFFCENTER_RIGHT_BITFIELD) {
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_OFF_CENTER_RIGHT; 
              PostNavigateService(NewEvent);
          } else if (tapeState == OFFCENTER_LEFT_BITFIELD) {
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_OFF_CENTER_LEFT; 
              PostNavigateService(NewEvent);
          } else if (tapeState == LEFT_TWO_BITS || tapeState == LEFT_BIT_ONLY) {
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_EXTREME_OFF_CENTER_LEFT; 
              PostNavigateService(NewEvent);
          } else if (tapeState == RIGHT_TWO_BITS || tapeState == RIGHT_BIT_ONLY) {
              NewEvent.EventType = ES_TAPE_DETECT;
              NewEvent.EventParam = TAPE_EXTREME_OFF_CENTER_RIGHT ;
              PostNavigateService(NewEvent);
          }
          
      break;
    }
    case ES_TIMEOUT:
    {
        if (ThisEvent.EventParam == LINE_TIMER) {
            DB_printf("Line timer timeout, resetting T count \n");
            T_count = 0;
            ES_Timer_StopTimer(LINE_TIMER);
        }
        break;
    }
  }
  return ReturnEvent;
}
