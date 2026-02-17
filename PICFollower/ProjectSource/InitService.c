/****************************************************************************
 Module
   InitService.c

 Revision
   1.0.0

 Description
   Init State Machine
****************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include "ES_Framework.h"
#include "InitService.h"

/*---------------------------- Module Types -------------------------------*/

typedef enum {
  IDLE,
  INIT_ORIENT,
  DETERMINE_SIDE,
  DEBUG
} InitState_t;

/*---------------------------- Module Variables --------------------------*/

static uint8_t MyPriority;
static InitState_t curState;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitInitService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
****************************************************************************/
bool InitInitService(uint8_t Priority)
{
  MyPriority = Priority;
  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_INIT;
  //curState = IDLE;
  curState = DEBUG;
  DB_printf("Init Service Initialized! \r\n");

  if (ES_PostToService(MyPriority, ThisEvent) == true) {
    return true;
  } else {
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

bool PostInitService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunNavigateService
****************************************************************************/

ES_Event_t RunInitService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (curState) {

    case DEBUG:

      DB_printf("\r\n");
      break;

    case IDLE:

      if (ThisEvent.EventType == ES_ENTRY) {
        // all motors off
        // indicators off
      }

      if (ThisEvent.EventType == ES_START_BUTTON) {
        curState = INIT_ORIENT;
      }

      break;


    case INIT_ORIENT:

      if (ThisEvent.EventType == ES_ENTRY) {
        // start game timer
        // turn CC slowly
      }

      if (ThisEvent.EventType == ES_BOTH_ULTRASONIC_ACTIVE) {
        curState = DETERMINE_SIDE;
      }

      if (ThisEvent.EventType == ES_GAME_TIMER_EXPIRED) {
        curState = IDLE;
      }

      if (ThisEvent.EventType == ES_EXIT) {
        // stop motors
      }

      break;


    case DETERMINE_SIDE:

      if (ThisEvent.EventType == ES_ENTRY) {
        // turn CC slowly
      }

      if (ThisEvent.EventType == ES_BEACON_DETECTED) {

        ES_Event_t SideEvent;
        SideEvent.EventType = ES_SIDE_INDICATED;

        if (ThisEvent.EventParam == LEFT_BEACON_FREQ) {
          SideEvent.EventParam = LEFT;
          // move indicator to left
        } else {
          SideEvent.EventParam = RIGHT;
          // move indicator to right
        }

        ES_PostAll(SideEvent);

        // transition into Navigate SM implicitly handled by HSM
      }

      if (ThisEvent.EventType == ES_GAME_TIMER_EXPIRED) {
        curState = IDLE;
      }

      break;
  }

  return ReturnEvent;
}
