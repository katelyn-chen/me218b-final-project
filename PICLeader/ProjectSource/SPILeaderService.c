/****************************************************************************
  Module
    SPILeaderService.c 

  Summary
    - Configure SPI in leader (master) mode using the SPI HAL
    - Periodically query the Follower
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

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


/*----------------------------- Module Defines ----------------------------*/
/* use the symbolic timer name from ES_Configure.h */
#ifndef SPI_TIMER
#define SPI_TIMER            0u
#endif

/* how often to ask for a new command (ms) */
#define SPI_POLL_MS          50u

/* SPI module selection */
#define CG_SPI_MODULE        SPI_SPI1

/* SPI clock period (ns). 500 kHz -> 2000 ns period */
//#define SPI_CLK_PERIOD_NS    2000u
#define SPI_CLK_PERIOD_NS      50000u

/* ---------------- Command Generator bytes (Appendix A) ----------------
   keeping these here so SPIService.c is self-contained.
----------------------------------------------------------------------- */
// Still need to add a few more commands
#define CMD_STOP                  0x00
#define CMD_TRANS_FWD             0x01
#define CMD_TRANS_BWD             0x02
#define CMD_ROT_CW_90             0x03
#define CMD_ROT_CCW_90            0x04
#define CMD_ROT_CCW_45            0x05
#define CMD_ROT_CW_45             0x06
#define CMD_TAPE_T_DETECT         0x07
#define CMD_LINE_FOLLOW           0x08
#define CMD_GET_BEACON_FREQ       0x11
#define CMD_INIT_ORIENT           0x12
#define CMD_SIDE_FOUND_RIGHT      0x13
#define CMD_SIDE_FOUND_LEFT       0x14
#define CMD_BEACON_R_FOUND        0x15
#define CMD_BEACON_L_FOUND        0x16
#define CMD_BEACON_G_FOUND        0x17
#define CMD_BEACON_B_FOUND        0x18
#define CMD_END_GAME              0x99
#define CMD_QUERY                 0xAA
#define CMD_NOOP                  0xFF
#define CMD_TESTING               0x10  // checkpoint 2 cmd

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  SEND,
  RECEIVE,
  DEBUG,
  IDLE
} LeaderState_t;

typedef enum {
    INIT,
    LOCATE_BEACON,
    BEACON_FOUND
} DebugState_t;

typedef enum {
    LEFT,
    RIGHT
} SideIndicated_t;

/*---------------------------- Module Variables --------------------------*/
static uint8_t MyPriority;
LeaderState_t curState;
DebugState_t debugState;
static uint8_t curCmd;
uint8_t prevCmd;
static uint8_t outCmd;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void InitPinHardware(void);
static uint8_t Follower_QueryByte(uint8_t outByte);
static void HandleCommandByte(uint8_t cmd);
static bool IsKnownCommand(uint8_t cmd);

/*------------------------------ Module Code ------------------------------*/
bool InitSPILeaderService(uint8_t Priority)
{
  MyPriority = Priority;
  curState = DEBUG;
  curCmd = CMD_TRANS_FWD;
  debugState = INIT;
  //curCmd = CMD_GET_BEACON_FREQ;
  InitSPIHardware();
  InitPinHardware();
  DB_printf("SPILeaderService initialized\r\n");

  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_INIT;
  ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS); // comment out later for button
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostSPILeaderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPILeaderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;
  switch (ThisEvent.EventType) {
    case ES_INIT_GAME:
        // tell follower to start!
        curCmd = CMD_INIT_ORIENT;
        curState = SEND;
        ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);     // init SPI timer
        break;

    case ES_END_GAME: {
        // tell follower to stop
        SPIOperate_SPI1_Send8Wait(CMD_END_GAME);
        ES_Timer_StopTimer(SPI_TIMER);
        // stop all servos
        break;
    }

    case ES_TIMEOUT:
        if (ThisEvent.EventParam == SPI_TIMER) {
            switch (curState) {
                case DEBUG:
                {
                    switch (debugState) {
                        case INIT:
                        {
                            uint8_t followerData;
                            curCmd = CMD_GET_BEACON_FREQ;
                            DB_printf("Leader sending get beacon freq command: %d\r\n", curCmd);
                            followerData = Follower_QueryByte(curCmd);
                            debugState = LOCATE_BEACON;
                            break;
                        }
                        
                        case LOCATE_BEACON: 
                        {
                            uint8_t followerData;
                            DB_printf("Leader sending query command! %d\n", curCmd);
                            followerData = Follower_QueryByte(CMD_QUERY);
                            
                            if (!IsKnownCommand(followerData))
                            {
                                DB_printf("SPIService: unknown cmd 0x0%d\r\n", followerData);
                            }

                            HandleCommandByte(followerData);
                            break;
                        }
                        
                        case BEACON_FOUND:
                        {
                            curCmd = CMD_TRANS_FWD;
                            DB_printf("Leader sending get fwd command: %d\r\n", curCmd);
                            SPIOperate_SPI1_Send8Wait(curCmd);
                            break;
                        }
                        
                    }
                    ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
                }
                case SEND:
                {
                    //curCmd = CMD_TRANS_FWD;
                    //DB_printf("Leader sending forward command! %d\n", curCmd);
                    DB_printf("Sending cur command: %d\r\n", curCmd);
                    SPIOperate_SPI1_Send8Wait(curCmd);
                    ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
                    //curState = RECEIVE;
                    break;
                }
                
                case RECEIVE:
                {
                    uint8_t followerData;
                    //DB_printf("Leader sending query command! %d\n", curCmd);
                    followerData = Follower_QueryByte(CMD_QUERY);

                    /* ignore unknown bytes (do not silently convert) */
                    if (!IsKnownCommand(followerData))
                    {
                        DB_printf("SPIService: unknown cmd 0x0%d\r\n", followerData);
                        ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
                        return ReturnEvent;
                    }

                    HandleCommandByte(followerData);
                    curState = IDLE;
                    break;
                }

                case IDLE:
                {
                    //curState = SEND;
                    break;
                }
        }
        ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
        }
        
    }

  return ReturnEvent;
}

/*=========================== SPI HELPERS ============================*/
static void InitSPIHardware(void)
{
  DB_printf("Leader SPI Setup\n"); 
  /* HAL SPI setup */
  SPISetup_BasicConfig(CG_SPI_MODULE);
  SPISetup_SetLeader(CG_SPI_MODULE, SPI_SMP_MID);

   /* map data pins */
  SPISetup_MapSDOutput(CG_SPI_MODULE, SPI_RPB5);
  
  SPISetup_MapSDInput(CG_SPI_MODULE,  SPI_RPB8);
  SPISetup_MapSSOutput(CG_SPI_MODULE, SPI_RPB15);

  /* clock mode settings */
  SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);    // CKP = 1
  SPISetup_SetActiveEdge(CG_SPI_MODULE, SPI_SECOND_EDGE);   // CKE = 0
  
  SPISetEnhancedBuffer(CG_SPI_MODULE, true);
  SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

  /* set SPI bit time (ns) */
  SPISetup_SetBitTime(CG_SPI_MODULE, SPI_CLK_PERIOD_NS);    // 1200?

  SPISetup_EnableSPI(CG_SPI_MODULE);

  /* clear any stale data */
  SPIOperate_ReadData(CG_SPI_MODULE);
}

static uint8_t Follower_QueryByte(uint8_t outByte)
{
  SPIOperate_SPI1_Send8Wait(outByte);
  return SPIOperate_ReadData(CG_SPI_MODULE);
}

/*======================= COMMAND -> EVENT MAP =======================*/
static bool IsKnownCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case CMD_STOP:
        case CMD_TRANS_FWD:
        case CMD_TRANS_BWD:
        case CMD_ROT_CW_90:
        case CMD_ROT_CCW_90:
        case CMD_ROT_CCW_45:
        case CMD_ROT_CW_45:
        case CMD_TAPE_T_DETECT:
        case CMD_LINE_FOLLOW:
        case CMD_GET_BEACON_FREQ:
        case CMD_QUERY:
        case CMD_NOOP:
        case CMD_TESTING:
        case CMD_SIDE_FOUND_RIGHT:
        case CMD_SIDE_FOUND_LEFT:
        case CMD_BEACON_R_FOUND:
        case CMD_BEACON_L_FOUND:
        case CMD_BEACON_G_FOUND:
        case CMD_BEACON_B_FOUND:
            return true;

        default:
            return false;
    }
}

static void HandleCommandByte(uint8_t cmd)
{
  if (cmd != prevCmd) {
    outCmd = CMD_NOOP;
    DB_printf("Leader received byte from follower: %d.\r\n", cmd);
    switch (cmd)
    {
      case CMD_TRANS_FWD: {
        DB_printf("Received forward command \r\n");
        break;
    }

      case CMD_STOP: {
        DB_printf("Received STOP command\r\n");
        break;
    }

    case CMD_TRANS_BWD: {
        DB_printf("Received backwards command\r\n");
        break;
    }

    case CMD_ROT_CW_90: {
        DB_printf("Received CW 90 rotation command\r\n");
        break;
    }

    case CMD_ROT_CCW_90: {
        DB_printf("Received CCW 90 rotation command\r\n");
        break;
    }


    case CMD_ROT_CW_45: {
        DB_printf("Received CW 45 rotation command\r\n");
        break;
    }

    case CMD_ROT_CCW_45: {
        DB_printf("Received CCW 45 rotation command\r\n");
        break;
    }

    case CMD_TAPE_T_DETECT: {
        DB_printf("Received TAPE detect command\r\n");
        break;
    }

    case CMD_LINE_FOLLOW: {
        DB_printf("Received LINE FOLLOW command\r\n");
        break;
    }

    case CMD_GET_BEACON_FREQ: {
        DB_printf("Received GET BEACON FREQ command\r\n");
        break;
    }

    case CMD_TESTING: {
        DB_printf("Received TESTING command and sending BEACON\r\n");
        outCmd = CMD_GET_BEACON_FREQ;
        //outCmd = CMD_ROT_CW_90;
        break;
    }

    case CMD_SIDE_FOUND_RIGHT: {
        ES_Event_t SideEvent;
        SideEvent.EventType = ES_SIDE_INDICATED;
        SideEvent.EventParam = RIGHT;
        ES_PostAll(SideEvent);
    }

    case CMD_SIDE_FOUND_LEFT: {
        ES_Event_t SideEvent;
        SideEvent.EventType = ES_SIDE_INDICATED;
        SideEvent.EventParam = LEFT;
        ES_PostAll(SideEvent);
    }
    
    case CMD_BEACON_R_FOUND: {
        DB_printf("Received BEACON R FOUND command\r\n");
        debugState = BEACON_FOUND;
    }
    
    case CMD_BEACON_B_FOUND: {
        DB_printf("Received BEACON B FOUND command\r\n");
        debugState = BEACON_FOUND;
    }
    
    case CMD_BEACON_L_FOUND: {
        DB_printf("Received BEACON L FOUND command\r\n");
        debugState = BEACON_FOUND;
    }
    
    case CMD_BEACON_G_FOUND: {
        DB_printf("Received BEACON G FOUND command\r\n");
        debugState = BEACON_FOUND;
    }
      default:
        break;
        
    }
    
    if (outCmd != CMD_NOOP) {
       curCmd = outCmd;
       DB_printf("Setting cur to out command: %d\r\n", outCmd);
       //SPIOperate_SPI1_Send8Wait(outCmd);
       curState = SEND;
    }
    
    prevCmd = cmd;
  } 
}

static void InitPinHardware(void)
{
    // Capacitive Touch Sensor: RA2, Pin 9
    PIN_MapPinInput(InitButton);
}