/****************************************************************************
 Module
   BeaconService.c

 Description
   Detects IR beacon signals and posts events to motor service
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BeaconService.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/

bool InitBeaconService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;
  ThisEvent.EventType = ES_INIT;

  // set up IR pin as input

  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool PostBeaconService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}


ES_Event_t RunBeaconService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  // if ES_BEACON_DETECTED event
    if (ThisEvent.EventType == ES_BEACON_FOUND)
    {
        // create new event to post to motor service
        ES_Event_t NewEvent;
        NewEvent.EventType = ES_BEACON_FOUND;
        // post to motor service
        PostMotorService(NewEvent);
    }
  // post to motor service
  return ReturnEvent;
}



/*------------------------------ End of file ------------------------------*/

