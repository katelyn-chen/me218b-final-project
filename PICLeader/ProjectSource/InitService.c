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
#include "SPILeaderService.h"
#include "CollectService.h"
#include "dbprintf.h"

/*---------------------------- Module Types -------------------------------*/

typedef enum {
  IDLE,
  GAME_MODE
} InitState_t;

/*---------------------------- Module Variables --------------------------*/

static uint8_t MyPriority;
static InitState_t curState;
static uint16_t SecondsElapsed;   // counter for game duration

static bool CollectDebugStarted = false;
static bool EncoderDebugStart = true;

/*---------------------------- Module Defines --------------------------*/
#define   GAME_TIME_MS    1000    // 1 second timer
#define   GAME_DURATION   138     // 2 minutes 18 seconds = 138 seconds

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
  curState = IDLE;
  DB_printf("Init Service Initialized! \r\n");

  if (ES_PostToService(MyPriority, ThisEvent) == true) {
    return true;
  } else {
      return false;
  }
}

/****************************************************************************
 Function
    PostInitService

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
    RunInitService
****************************************************************************/

ES_Event_t RunInitService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (curState) {
    case IDLE:
      if (ThisEvent.EventType == ES_INIT) {
            DB_printf("InitService received ES_INIT\r\n");

            /*
              DEBUG: start CollectService immediately on boot.
              This is only for bench testing. Remove when integrating full game logic.
            */
            if (CollectDebugStarted == false) {
              CollectDebugStarted = true;

              //ES_Event_t StartCollect;
              //StartCollect.EventType = ES_COLLECT_START;
              //StartCollect.EventParam = 0;
              //PostCollectService(StartCollect);

              //DB_printf("InitService: posted ES_COLLECT_START (debug)\r\n");
            }
            
            /*
             Debug encoder service*/
//            if (EncoderDebugStart) {
//                DB_printf("entering encoder\r\n");
//
//                ES_Event_t encoder;
//                encoder.EventType = ES_ENCODER_TARGET_ROT;
//                PostEncoderService(encoder);
//            }
      }

      if (ThisEvent.EventType == ES_ENTER_IDLE) {
        // send motor stop command to follower to stop motors
        // indicators off
        ReturnEvent.EventType = ES_END_GAME;
        ES_PostAll(ReturnEvent);
      }

      if (ThisEvent.EventType == ES_START_BUTTON) {
        DB_printf("Start button has been pressed!\r\n");
        ES_Event_t NewEvent;
        NewEvent.EventType = ES_INIT_GAME;
        ES_PostAll(NewEvent);
        curState = GAME_MODE;
      }
      break;


    case GAME_MODE:
      if (ThisEvent.EventType == ES_INIT_GAME) {
        LATAbits.LATA3 = 1;
        SecondsElapsed = 0;  // reset counter
        ES_Timer_InitTimer(GAME_TIMER, GAME_TIME_MS); // start game timer
      }

      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == GAME_TIMER) {
        SecondsElapsed++;

        if (SecondsElapsed >= GAME_DURATION) {
          ES_Timer_StopTimer(GAME_TIMER); // stop game timer
          curState = IDLE;
          ES_Event_t GameOver;
          GameOver.EventType = ES_ENTER_IDLE;
          LATAbits.LATA3 = 0;
          PostInitService(GameOver);
        } else {
          ES_Timer_InitTimer(GAME_TIMER, GAME_TIME_MS); // restart 1 second timer
        }
      }
      break;
  }

  return ReturnEvent;
}