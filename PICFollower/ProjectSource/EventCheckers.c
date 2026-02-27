/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"
#include "PIC32_AD_Lib.h"
#include "PIC32_SPI_HAL.h"
#include "SPIFollowerService.h"


// This is the event checking function sample. It is not intended to be
// included in the module. It is only here as a sample to guide you in writing
// your own event checkers
#if 0
/****************************************************************************
 Function
   Check4Lock
 Parameters
   None
 Returns
   bool: true if a new event was detected
 Description
   Sample event checker grabbed from the simple lock state machine example
 Notes
   will not compile, sample only
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Lock(void)
{
  static uint8_t  LastPinState = 0;
  uint8_t         CurrentPinState;
  bool            ReturnVal = false;

  CurrentPinState = LOCK_PIN;
  // check for pin high AND different from last time
  // do the check for difference first so that you don't bother with a test
  // of a port/variable that is not going to matter, since it hasn't changed
  if ((CurrentPinState != LastPinState) &&
      (CurrentPinState == LOCK_PIN_HI)) // event detected, so post detected event
  {
    ES_Event ThisEvent;
    ThisEvent.EventType   = ES_LOCK;
    ThisEvent.EventParam  = 1;
    // this could be any of the service post functions, ES_PostListx or
    // ES_PostAll functions
    ES_PostAll(ThisEvent);
    ReturnVal = true;
  }
  LastPinState = CurrentPinState; // update the state for next time

  return ReturnVal;
}

#endif

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType   = ES_NEW_KEY;
    ThisEvent.EventParam  = GetNewKey();
    ES_PostAll(ThisEvent);
    return true;
  }
  return false;
}



// check if ultrasonic distance has been measured -- potentially switch to service with IC?
bool Check4Wall(void){
    bool ReturnVal = false; 
    bool CurrentEchoState = PIN_ReadDigitalPIC32Pin(UltrasonicEcho); // Read the current state of the Echo pin
    static bool LastEchoState = 0;
    static uint8_t TimeAtRise = 0;
    
    if (CurrentEchoState != LastEchoState) {
        if (CurrentEchoState == 1) {
            // start timer when echo is high (start of pulse)
            TimeAtRise = ES_Timer_GetTime();
        } else {
            // echo low, calculate pulse width and convert to distance
            uint32_t TimeDelta = ES_Timer_GetTime() - TimeAtRise;
            
            // assuming 1 tick = 1 microsecond
            uint16_t DistanceCM = (uint16_t)(TimeDelta / 58);

            ES_Event_t newEvent;
            newEvent.EventType = ES_NEW_DIST; 
            newEvent.EventParam = DistanceCM;
            
            ES_PostAll(newEvent);
            
            ReturnVal = true;
        }
    }
    LastEchoState = CurrentEchoState;
    return ReturnVal;
}


/*
#define TapeSensor1 SPI_RPB4
#define TapeSensor2 SPI_RPB5
#define TapeSensor3  SPI_RPB11
#define TapeSensor4 SPI_RPB12
#define TapeSensor5 SPI_RPB13
*/
bool Check4Tape(void)
{
  bool tapeDetected = false;
  static uint8_t lastTapeState = 0;
  uint8_t currentTapeState = 0;

  currentTapeState |= (PORTBbits.RB4 == 1) << 0; // active high
  currentTapeState |= (PORTBbits.RB5 == 1) << 1;
  currentTapeState |= (PORTBbits.RB11 == 1) << 2;
  currentTapeState |= (PORTBbits.RB12 == 1) << 3;
  currentTapeState |= (PORTBbits.RB13 == 1) << 4;

  if (currentTapeState != lastTapeState && currentTapeState != 0) {
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_TAPE_CHANGE;
    ThisEvent.EventParam = currentTapeState; // bitfield of all sensors
    PostReflectiveSenseService(ThisEvent);
    tapeDetected = true;
  }

  lastTapeState = currentTapeState; 
  return tapeDetected;
}