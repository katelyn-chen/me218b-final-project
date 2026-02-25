/****************************************************************************
 Module
   EncoderService.c

 Revision
   1.0.1

 Description
   This is a EncoderService. The provided encoders give 12 counts per revolution
   of the motor shaft

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from EncoderFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <stdbool.h> 
#include <stdio.h>
#include <sys/attribs.h>
#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "EncoderService.h"
#include "PIC32_SPI_HAL.h"
#include "SPILeaderService.h"
/*----------------------------- Module Defines ----------------------------*/
#define COUNTS_PER_REV 12.0
#define GEAR_RATIO     27.0   // needs to be verified
#define WHEEL_DIAMETER 0.1    // meters (10 cm)
#define PI 3.14159265359
#define ENCODER_CHECK_MS 20
#define KP 0.1f

/*----------------------------- Module Types ----------------------------*/
typedef enum {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_CW,
  MOTOR_CCW
} MotorState_t;

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void InitPinHardware();
void InitTimer3();
void InitIC3();
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static volatile int32_t LeftCount = 0;
static volatile int32_t RightCount = 0;

static int32_t StartLeftCount = 0;
static int32_t StartRightCount = 0;

static int32_t TargetDelta = 0;

static bool MoveActive = false;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitEncoderService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitEncoderService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;

  InitPinHardware();
  InitTimer3();
  InitIC1();
  InitIC3();
  __builtin_enable_interrupts();
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostEncoderService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostEncoderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEncoderService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunEncoderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  switch(ThisEvent.EventType)
  {
    case ES_INIT:
        {
            MoveActive = false;
        }
        break;

    case ES_ENCODER_TARGET:
      {
          TargetDelta = (int16_t)ThisEvent.EventParam;

          StartLeftCount = LeftCount;
          StartRightCount = RightCount;

          MoveActive = true;
          ES_Timer_InitTimer(ENCODER_TIMER, ENCODER_CHECK_MS);
      break;
      }

    case ES_TIMEOUT:
      {
          if(MoveActive) {
              int32_t leftTravel  = LeftCount  - StartLeftCount;
              int32_t rightTravel = RightCount - StartRightCount;

              int32_t avgTravel = (leftTravel + rightTravel) / 2;

              // Handle forward AND backward motion
              if( ABS(avgTravel) >= ABS(TargetDelta) ) {
                  MoveActive = false;

                  ES_Event_t doneEvent;
                  doneEvent.EventType = ES_ENCODER_TARGET_REACHED;
                  doneEvent.EventParam = 0;

                  PostSPILeaderService(doneEvent);
                  ES_Timer_StopTimer(ENCODER_TIMER);
              } else {
                ES_Timer_InitTimer(ENCODER_TIMER, ENCODER_CHECK_MS);
              }
          }
      break;
      }
    }

  

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/*
  Pin Hardware Mapping:
    RB11 -> IC3 -> LeftEncoderA
    RB9  -> LeftEncoderB
    RB13 -> IC1 -> RightEncoderA
    RB3  -> RightEncoderB

*/

void InitPinHardware(void)
{
  // Sets the TRIS bits = 1 and makes pins digital
  PIN_MapPinInput(LeftEncoderA);
  PIN_MapPinInput(LeftEncoderB);
  PIN_MapPinInput(RightEncoderA);
  PIN_MapPinInput(RightEncoderB);

  // PPS Mapping
  IC3R = 0b0011;
  IC1R = 0b0011;
}

/* This function sets up timer 3 for interrupts to be used as input control
 * timer
 */
void InitTimer3() {
    T3CONbits.ON = 0;            // disable timer
    T3CONbits.TCS = 0;           // clear to select internal PBCLK source
    T3CONbits.TCKPS = 0b0011;    // choosing 1:8 prescale value
    TMR3 = 0;                    // clearing timer register
    PR3 = 0xFFFF;
    IFS0CLR = _IFS0_T3IF_MASK;  // clearing interrupt flag
    T3CONbits.ON = 1;          // enable timer
}

/* This function sets up interrupt capture 1
 */
void InitIC1() {
  IC1CONbits.ON = 0;           // disable module
  IC1CONbits.SIDL = 0;         // keep operating in idle mode
  IC1CONbits.C32 = 0;          // use 16 bit timer
  IC1CONbits.ICTMR = 0;        // use Timer3
  IC1CONbits.ICI = 0b0000;     // Interrupt on every capture event
  IC1CONbits.ICM = 0b0011;     // capture every rising edge
  IFS0CLR = _IFS0_IC1IF_MASK;  // clear flag
  IEC0SET = _IEC0_IC1IE_MASK;  // enable interrupt
  IPC1bits.IC1IP = 7;          // priority (must match IPL7)
  IPC1bits.IC1IS = 0;          // sub-priority
  IC1CONbits.ON = 1;           // enable module
}

/* This function sets up interrupt capture 3
 */
void InitIC3() {
  IC3CONbits.ON = 0;           // disable module
  IC3CONbits.SIDL = 0;         // keep operating in idle mode
  IC3CONbits.C32 = 0;          // use 16 bit timer
  IC3CONbits.ICTMR = 0;        // use Timer3
  IC3CONbits.ICI = 0b0000;     // Interrupt on every capture event
  IC3CONbits.ICM = 0b0011;     // capture every rising edge
  IFS0CLR = _IFS0_IC3IF_MASK;  // clear flag
  IEC0SET = _IEC0_IC3IE_MASK;  // enable interrupt
  IPC3bits.IC3IP = 7;          // priority (must match IPL7)
  IPC3bits.IC3IS = 0;          // sub-priority
  IC3CONbits.ON = 1;           // enable module
}

 // Left Motor ISR
 void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL7SOFT) IC3_ISR(void)
{
    uint16_t capture = IC3BUF;

    while(IC3CONbits.ICBNE) {
        capture = IC3BUF;
    }

    // Read B channel to determine direction
    if(PORTBbits.RB9 == 1) {
        LeftCount++;    // Forward
    } else {
        LeftCount--;    // Reverse
    }

    IFS0CLR = _IFS0_IC3IF_MASK;
}

 // Right Motor ISR
 void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL7SOFT) IC1_ISR(void)
{
    uint16_t capture = IC1BUF;

    while(IC1CONbits.ICBNE) {
        capture = IC1BUF;
    }

    // Read B channel
    if(PORTBbits.RB3 == 1) {
        RightCount++;
    } else {
        RightCount--;
    }

    IFS0CLR = _IFS0_IC1IF_MASK;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

