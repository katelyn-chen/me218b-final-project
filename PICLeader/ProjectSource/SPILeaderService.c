/****************************************************************************
  Module
    SPILeaderService.c 

  Summary
    - Configure SPI in leader (master) mode using the SPI HAL
    - Send motor / mode commands to the Follower PIC
    - Periodically query the Follower for 1-byte latched status
    - Convert follower status bytes into framework events on PIC1

  Notes
    - Follower returns a latched status byte on every SPI transfer
    - Status gets cleared on the follower only after CMD_QUERY
    - This service keeps polling with CMD_QUERY when no new command is pending

  Wiring
    MOSI: Leader SDO (RB5, pin 14) --> Follower SDI (RA0, pin 2)
    MISO: Leader SDI (RB8, pin 17) --> Follower SDO (RA3, pin 10)
    Leader SCK (RB14, pin 25) --> Follower SCK1  (RB14, pin 25)
    Leader SS (RB15, pin 26) --> Follower CS (RB15, pin 26)
****************************************************************************/

#include "SPILeaderService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "PIC32_SPI_HAL.h"
#include "EncoderService.h"
#include "DispenseService.h"
#include "CollectService.h"

/*----------------------------- Module Defines ----------------------------*/
/* use the symbolic timer name from ES_Configure.h */
#ifndef SPI_TIMER
#define SPI_TIMER            0u
#endif

/* how often to send SPI traffic (ms) */
#define SPI_POLL_MS          50u

/* SPI module selection */
#define CG_SPI_MODULE        SPI_SPI1

#define SPI_CLK_PERIOD_NS      50000u

/* ENCODER SERVICE PULSE VALUES */
#define ROT_90_PULSES           25
#define ROT_180_PULSES          ROT_90_PULSES*3
#define COLLECT_REV_ALIGN       10      // FIRST reverse from dispenser before lowering arm 
#define INIT_ROT_ADJUST         23      // first rotation after seeing beacon
#define RIGHT_FULL_ROTATE       60      // tuned! one full wheel rotation
#define SIDE_INDICATE_FIRST_FWD 25      // first fwd towards beacon
#define COLLECT_NUDGE_FWD       18      // nudging fwd to dispenser
#define COLLECT_NUDGE_BACK      15      // nudging back from dispenser
#define COLLECT_FWD_AFTER_T     15      // moving forward after the T

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  LEADER_IDLE = 0,
  LEADER_SEND_PENDING,
  LEADER_QUERY_STATUS
} LeaderState_t;

typedef enum {
  LEFT  = 0,
  RIGHT = 1
} SideIndicated_t;

/*---------------------------- Module Variables --------------------------*/
static uint8_t       MyPriority;
static LeaderState_t curState;

static uint8_t PendingCmd;
static bool    HasPendingCmd;

static uint8_t PrevFollowerStatus;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void InitPinHardware(void);

static uint8_t XferByte(uint8_t outByte);
static void HandleFollowerStatus(uint8_t statusByte);

static bool IsKnownFollowerStatus(uint8_t cmd);
static void QueueCommand(uint8_t cmd);

/*------------------------------ Module Code ------------------------------*/
bool InitSPILeaderService(uint8_t Priority)
{
  MyPriority = Priority;

  curState = LEADER_QUERY_STATUS;
  PendingCmd = CMD_NOOP;
  HasPendingCmd = false;
  PrevFollowerStatus = CMD_NOOP;

  InitSPIHardware();
  InitPinHardware();

  DB_printf("SPILeaderService initialized\r\n");

  ES_Event_t ThisEvent;
  ThisEvent.EventType  = ES_INIT;
  ThisEvent.EventParam = 0;

  ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostSPILeaderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPILeaderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (ThisEvent.EventType)
  {
    case ES_INIT_GAME:
    {
      /*
       This starts the game and queues the first command to the follower,
       * CMD_INIT_ORIENT which tells the bot to initially orient to determine 
       * side. This case also starts the SPI timer to regularly send SPI comm to
       * the follower
      */
      DB_printf("Queueing command CMD INIT ORIENT\r\n");
      QueueCommand(CMD_INIT_ORIENT);
      ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
      break;
    }

    case ES_END_GAME:
    {
      /*
       This sends the end game command to the follower and stops the SPI timer
       * to end all communication to the follower
      */
      (void)XferByte(CMD_END_GAME);
      ES_Timer_StopTimer(SPI_TIMER);
      break;
    }

    case ES_CMD_REQ:
    {
      /*
       This reacts to events posted by other services on the leader PIC to queue
       * a command to be sent to the follower.
      */
      uint8_t cmd = (uint8_t)ThisEvent.EventParam;
      QueueCommand(cmd);
      break;
    }
    case ES_ENCODER_TARGET_REACHED:
      /*
       Once the target number of pulses have been reached by the encoder service, it
       * posts an event to the leader SPI Service to send to the follower.
      */
      QueueCommand(CMD_MOVE_DONE);
      break;

    case ES_TIMEOUT:
    {
      /*
       This case handles what to do when the SPI timer times out. If there is a 
       * command to be sent, it changes the state to actively sending a command.
       * Otherwise, the state becomes a query state where the leader reads in
       * data from the follower after sending a "query" byte.
      */
      if (ThisEvent.EventParam != SPI_TIMER) break;
      if (HasPendingCmd)
      {
        curState = LEADER_SEND_PENDING;
        DB_printf("Has pending cmd\r\n");
        
      }
      else
      {
        curState = LEADER_QUERY_STATUS;
      }

      if (curState == LEADER_SEND_PENDING)
      {
        uint8_t status;
        status = XferByte(PendingCmd);
        DB_printf("Has pending cmd, pending cmd status: %d\r\n", status);
        HasPendingCmd = false;
        PendingCmd = CMD_NOOP;

        if (IsKnownFollowerStatus(status))
        {
          HandleFollowerStatus(status);
        } else {
            DB_printf("Unknown command\r\n");
        }
      }
      else
      {
        uint8_t status = XferByte(CMD_QUERY);

        if (IsKnownFollowerStatus(status))
        {
          HandleFollowerStatus(status);
        }
      }

      ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
      break;
    }

    default:
      break;
  }

  return ReturnEvent;
}

/*=========================== SPI HELPERS ============================*/
static void InitSPIHardware(void)
{
  DB_printf("Leader SPI Setup\r\n");

  /* HAL SPI setup */
  SPISetup_BasicConfig(CG_SPI_MODULE);
  SPISetup_SetLeader(CG_SPI_MODULE, SPI_SMP_MID);

  /* map data pins */
  SPISetup_MapSDOutput(CG_SPI_MODULE, SPI_RPB5);
  SPISetup_MapSDInput(CG_SPI_MODULE,  SPI_RPB8);
  SPISetup_MapSSOutput(CG_SPI_MODULE, SPI_RPB15);

  /* clock mode settings */
  SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);     // CKP = 1
  SPISetup_SetActiveEdge(CG_SPI_MODULE,     SPI_SECOND_EDGE);// CKE = 0

  SPISetEnhancedBuffer(CG_SPI_MODULE, true);
  SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

  /* set SPI bit time (ns) */
  SPISetup_SetBitTime(CG_SPI_MODULE, SPI_CLK_PERIOD_NS);

  SPISetup_EnableSPI(CG_SPI_MODULE);

  /* clear any stale data */
  SPIOperate_ReadData(CG_SPI_MODULE);
}

/*
 * This function uses the SPI HAL Send8Wait function to transfer data over SPI
 * communication. It returns the data read in from the follower.
*/
static uint8_t XferByte(uint8_t outByte)
{
    if (outByte!= CMD_QUERY) {
    }
  SPIOperate_SPI1_Send8Wait(outByte);
  return (uint8_t)SPIOperate_ReadData(CG_SPI_MODULE);
}

/*
 * This function adds a command to the queue and sets the HasPendingCmd flag to
 * be true to indicate to the leader that there is a command to be sent.
*/
static void QueueCommand(uint8_t cmd)
{
  /*
    queue is 1-deep on purpose
    latest command wins to avoid building up stale actions
  */
  PendingCmd = cmd;
  HasPendingCmd = true;
}

/*=========================== STATUS HANDLING ============================*/
/*
 * This function checks to make sure the command sent by the follower is a real
 * and existing command. If the command doesn't exist, the leader ignores it and
 * this function returns false.
*/
static bool IsKnownFollowerStatus(uint8_t cmd)
{
  switch (cmd)
  {
    case CMD_NOOP:
    case CMD_TESTING:
    case CMD_SIDE_FOUND_BLUE:
    case CMD_SIDE_FOUND_GREEN:
    case CMD_BEACON_R_FOUND:
    case CMD_BEACON_L_FOUND:
    case CMD_BEACON_G_FOUND:
    case CMD_BEACON_B_FOUND:
    case CMD_ENCODER_FIRST_ALIGN:
    case CMD_ROT_CCW_90:
    case CMD_ROT_CW_180:
    case CMD_ROT_CCW_180:
    case CMD_ROT_CW_90:
    case CMD_FIRST_COLLECT_START:
    case CMD_SECOND_COLLECT_START:
    case CMD_OTHER_COLLECT_START:
    case CMD_DISPENSE_START:
    case CMD_ENCODER_FIRST_FWD:
    case CMD_COLLECT_BACK:
    case CMD_COLLECT_FWD:
    case CMD_FIRST_COLLECT_ARM_UP:
    case CMD_FIRST_COLLECT_GRAB:
    case CMD_ALIGN_COLLECT:
    case CMD_FWD_AFTER_T:
      return true;

    default:
      return false;
  }
}

/*
 * This function responds to valid commands sent by the follower to the leader.
 * If there is an event that needs to be posted to other leader services in response
 * to the follower request, cmdEvent.EventType is modified and posted to the
 * desired service.
*/
static void HandleFollowerStatus(uint8_t statusByte)
{
  /*
    follower status bytes are latched until CMD_QUERY
    this guard avoids re-posting if the follower ever returns the same byte twice
  */
  ES_Event_t cmdEvent;

  if (statusByte == PrevFollowerStatus)
  {
    return;
  }
  PrevFollowerStatus = statusByte;

  switch (statusByte)
  {
    case CMD_NOOP:
      break;

    case CMD_SIDE_FOUND_BLUE:
    {
      // move indicator servo to blue
      DB_printf("Side indicated: BLUE\r\n");
      cmdEvent.EventType = ES_INDICATE_SIDE;
      cmdEvent.EventParam = FIELD_BLUE;
      PostDispenseService(cmdEvent);
      DB_printf("Moving towards L/R beacon! \r\n");
      ES_Event_t encoderEvent;
      encoderEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
      encoderEvent.EventParam = SIDE_INDICATE_FIRST_FWD;
      PostEncoderService(encoderEvent);
      break;
    }

    case CMD_SIDE_FOUND_GREEN:
    {
      // move indicator servo to green
      DB_printf("Side indicated: GREEN\r\n");
      cmdEvent.EventType = ES_INDICATE_SIDE;
      cmdEvent.EventParam = FIELD_GREEN;
      PostDispenseService(cmdEvent);
      ES_Event_t encoderEvent;
      encoderEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
      encoderEvent.EventParam = SIDE_INDICATE_FIRST_FWD;
      PostEncoderService(encoderEvent);
      break;
    }

    /*
      Beacon found status is mainly for debug / early alignment
    */
    case CMD_BEACON_R_FOUND:
      DB_printf("Follower status: BEACON R FOUND\r\n");
      break;

    case CMD_BEACON_L_FOUND:
      DB_printf("Follower status: BEACON L FOUND\r\n");
      break;

    case CMD_BEACON_G_FOUND:
      DB_printf("Follower status: BEACON G FOUND\r\n");
      break;

    case CMD_BEACON_B_FOUND:
      DB_printf("Follower status: BEACON B FOUND\r\n");
      break;

    case CMD_TESTING:
      DB_printf("Follower status: TESTING\r\n");
      break;

    case CMD_ROT_CCW_90:
      DB_printf("Follower status: ROTATING CCW 90\r\n");
      cmdEvent.EventType = ES_ENCODER_TARGET_ROT;
      cmdEvent.EventParam = ROT_90_PULSES;
      PostEncoderService(cmdEvent);
      break;
    
    case CMD_ENCODER_FIRST_ALIGN:
    /* This is when the bot is first turning to follow the tape! */
      DB_printf("Follower status: INIT rotate adjustment\r\n");
      cmdEvent.EventType = ES_ENCODER_TARGET_ROT;
      cmdEvent.EventParam = INIT_ROT_ADJUST;
      PostEncoderService(cmdEvent);
      break;

    case CMD_ALIGN_COLLECT:
    /* This is when the bot is facing the coal dispenser and needs to drive backwards */
      DB_printf("Follower status: Facing dispenser, aligning by moving fwd\r\n");
      cmdEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
      cmdEvent.EventParam = COLLECT_REV_ALIGN;
      PostEncoderService(cmdEvent);
      break;

    case CMD_FIRST_COLLECT_START:
      DB_printf("Initiating FIRST COLLECT process and posting to CollectService! \r\n");
      cmdEvent.EventType = ES_COLLECT_START;
      cmdEvent.EventParam = FIRST_COLLECT;
      PostCollectService(cmdEvent);
      break;
    
    case CMD_SECOND_COLLECT_START:
      DB_printf("Initiating SECOND COLLECT process and posting to CollectService! \r\n");
      cmdEvent.EventType = ES_COLLECT_START;
      cmdEvent.EventParam = SECOND_COLLECT;
      PostCollectService(cmdEvent);
      break;
    
    case CMD_OTHER_COLLECT_START:
      DB_printf("Initiating OTHER COLLECT process and posting to CollectService! \r\n");
      cmdEvent.EventType = ES_COLLECT_START;
      cmdEvent.EventParam = OTHER_COLLECT;
      PostCollectService(cmdEvent);
      break;
      
    case CMD_ENCODER_FIRST_FWD:
        DB_printf("Moving towards L/R beacon! \r\n");
        cmdEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
        cmdEvent.EventParam = SIDE_INDICATE_FIRST_FWD;
        PostEncoderService(cmdEvent);
        break;
    
    case CMD_COLLECT_FWD:
        DB_printf("Nudging forward to dispenser \r\n");
        cmdEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
        cmdEvent.EventParam = COLLECT_NUDGE_FWD;
        PostEncoderService(cmdEvent);
        break;
        
    case CMD_COLLECT_BACK:
        DB_printf("Nudging backward from dispenser \r\n");
        cmdEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
        cmdEvent.EventParam = COLLECT_NUDGE_BACK;
        PostEncoderService(cmdEvent);
        break;    
        
        
    case CMD_FIRST_COLLECT_ARM_UP:
        DB_printf("Telling collect service to move arm up\r\n");
        cmdEvent.EventType = ES_COLLECT_BACK_DONE;
        PostCollectService(cmdEvent);
        break;
          
      case CMD_FIRST_COLLECT_GRAB:
        DB_printf("Telling collect service to start grabbing again\r\n");
        cmdEvent.EventType = ES_COLLECT_FWD_DONE;
        PostCollectService(cmdEvent);
        break;
    
    case CMD_FWD_AFTER_T:
        DB_printf("Moving fwd after seeing the second tape\r\n");
        cmdEvent.EventType = ES_ENCODER_TARGET_STRAIGHT;
        cmdEvent.EventParam = COLLECT_FWD_AFTER_T;
        PostEncoderService(cmdEvent);
        break;
        
    case CMD_ROT_CW_180:
        DB_printf("Rotating 180 degrees clockwise\r\n");
        cmdEvent.EventType = ES_ENCODER_TARGET_ROT;
        cmdEvent.EventParam = ROT_180_PULSES;
        PostEncoderService(cmdEvent);
        break;
        
    case CMD_DISPENSE_START:
        cmdEvent.EventType = ES_DISPENSE_START;
        PostDispenseService(cmdEvent);
        break;
         

    default:
      break;
  }
}

static void InitPinHardware(void)
{
  /* Capacitive Touch Sensor: RA2, Pin 9 */
  PIN_MapPinInput(InitButton);
  PIN_MapPinOutput(ProgressLED);
  LATAbits.LATA3 = 0; // progress LED off
}