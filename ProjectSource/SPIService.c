/****************************************************************************
 Module
   SPIService.c

 Description
   Detects IR beacon signals and posts events to motor service
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

#include <xc.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "SPIService.h"
#include "PIC32_SPI_HAL.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/

bool InitSPIService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;
  ThisEvent.EventType = ES_INIT;

  // configure SPI1 module
  SPISetup_BasicConfig(SPI_SPI1);
  // set up SPI as follower
  SPISetup_SetLeader(SPI_SPI1, SPI_SMP_MID);

  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool PostSPIService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}


ES_Event_t RunSPIService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  if (ThisEvent.EventType == ES_INIT)
  {
      // any initialization code for service
      DB_printf("SPI Service Initialized\n");
  } 
  // if ES_BEACON_DETECTED event
    if (ThisEvent.EventType == ES_COMMAND_RECEIVED)
    {
        // based on each command, create new event to post to motor service
        switch (ThisEvent.EventParam)
        {
            // if 0x00 is received
                // stop, hold position, do not move or rotate
            // if 0x02 is received
                // rotate clockwise by 90 degrees (6 sec)
            // if 0x03 is received
                // rotate clockwise by 45 degrees (3 sec)
            // if 0x04 is received
                // rotate counterclockwise by 90 degrees (6 sec)
            // if 0x05 is received
                // rotate counterclockwise by 45 degrees (3 sec)
            // if 0x08 is received
                // move forward at half speed
            // if 0x09 is received
                // move forward at full speed
            // if 0x10 is received
                // move backward at half speed
            // if 0x11 is received
                // move backward at full speed
            // if 0x20 is received
                // align with beacon (5 sec)
            // if 0x40 is received
                // drive forward until tape detected
        }
    }
  return ReturnEvent;
}



/*------------------------------ End of file ------------------------------*/

