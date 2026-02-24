/****************************************************************************
  Module
    SPIFollowerService.c

  Summary
    - Configure SPI in follower (slave) mode using the SPI HAL
    - Periodically send to the Leader
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

  Wiring
    MOSI: Leader SDO (RB5, pin 14) --> Follower SDI (RA0, pin 2)
    MISO: Leader SDI (RB8, pin 17) --> Follower SDO (RA3, pin 10)
    Leader SCK (RB14, pin 25) --> Follower SCK1  (RB14, pin 25)
    Leader SS (RB15, pin 26) --> Follower CS (RB15, pin 26)
****************************************************************************/

#include "SPIFollowerService.h"
#include <sys/attribs.h>
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "PIC32_SPI_HAL.h"

#include "NavigateService.h"
#include "BeaconService.h"

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
#define SPI_CLK_PERIOD_NS    50000u
#define CMD_DELAY            100        // arbitrary delay amt, can change


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
  DEBUG
} FollowerState_t;

typedef enum {
    RIGHT,
    LEFT
} SideIndicate_t;
/*---------------------------- Module Variables --------------------------*/
static uint8_t MyPriority;
FollowerState_t curState;
static uint8_t curCmd;
static uint8_t prevCmd;
static volatile uint8_t incomingCmd;
static uint8_t outgoingCmd;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static uint8_t Leader_QueryByte(uint8_t outByte);
static void HandleCommandByte(uint8_t cmd);
static bool IsKnownCommand(uint8_t cmd);
static void InitPinHardware(void);

/*------------------------------ Module Code ------------------------------*/
bool InitSPIFollowerService(uint8_t Priority)
{
  MyPriority = Priority;
  curState = RECEIVE;
  InitSPIHardware();
  InitPinHardware();
  //ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
  DB_printf("SPIFollowerService initialized\r\n");

  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_INIT;
  ES_PostToService(MyPriority, ThisEvent);
  return true;
}

bool PostSPIFollowerService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent)
{
  //DB_printf("RunSPIFollowerService entered: event %d, param %d\r\n",
  //ThisEvent.EventType, ThisEvent.EventParam);
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };
    switch (ThisEvent.EventType) {

        case ES_SIDE_INDICATED:
        {
            if (ThisEvent.EventParam = RIGHT) {
                outgoingCmd = CMD_SIDE_FOUND_RIGHT;
                // somewhere we have to turn the indicator servo
            } else {
                outgoingCmd = CMD_SIDE_FOUND_LEFT;
                // somewhere we have to turn the indicator servo
            }
            break;
        }

        case ES_SPI_RECEIVED:
        {
            DB_printf("Received from byte leader %d\r\n", ThisEvent.EventParam);

            /* ignore unknown bytes */
            if (!IsKnownCommand(ThisEvent.EventParam))
            {
                DB_printf("SPIService: unknown leaderCmd 0x%02X\r\n", ThisEvent.EventParam);
                return ReturnEvent;
            }
            HandleCommandByte(ThisEvent.EventParam);
            break;
        }
        
        case ES_BEACON_FOUND:
        {
            uint8_t id = ThisEvent.EventParam;
            DB_printf("Beacon found! ID: %d\r\n", id);
            switch (id) {
                case BEACON_ID_G:
                    outgoingCmd = CMD_BEACON_G_FOUND;
                    break;
                    
                case BEACON_ID_B:
                    outgoingCmd = CMD_BEACON_B_FOUND;
                    break;
                    
                case BEACON_ID_R:
                    outgoingCmd = CMD_BEACON_L_FOUND;
                    break;
                    
                case BEACON_ID_L:
                    outgoingCmd = CMD_BEACON_L_FOUND;
                    break;                
            }
            break;
        }
            
            
    }
  return ReturnEvent;
}

/*=========================== SPI HELPERS ============================*/
static void InitSPIHardware(void)
{
/*  Wiring
    MOSI: Leader SDO (RB5, pin 14) --> Follower SDI (RA0, pin 2)
    MISO: Leader SDI (RB8, pin 17) --> Follower SDO (RA3, pin 10)
    Leader SCK (RB14, pin 25) --> Follower SCK1  (RB14, pin 25)
    Leader SS (RB15, pin 26) --> Follower CS (RB15, pin 26)*/

  DB_printf("Follower SPI Setup\n"); 
  /* HAL SPI setup */
  SPISetup_BasicConfig(CG_SPI_MODULE);
  SPISetup_SetFollower(CG_SPI_MODULE);

  // not sure if we need the two lines below but another team had them..
  ANSELBbits.ANSB14 = 0; // sets clock as input
  //TRISAbits.TRISRA3 = 0;  // SDO to output

   /* map data pins */
  SPISetup_MapSDOutput(CG_SPI_MODULE, SDOPin);
  SPISetup_MapSDInput(CG_SPI_MODULE,  SDIPin);
  SPISetup_MapSSInput(CG_SPI_MODULE, SSPin);

  /* clock mode settings */
  SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);    // CKP = 1
  SPISetup_SetActiveEdge(CG_SPI_MODULE, SPI_SECOND_EDGE);   // CKE = 0
  
  SPISetEnhancedBuffer(CG_SPI_MODULE, false);
  SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

  /* set SPI bit time (ns) */
  //SPISetup_SetBitTime(CG_SPI_MODULE, SPI_CLK_PERIOD_NS);
  SPISetup_EnableSPI(CG_SPI_MODULE);
  
  SPISetup_ConfigureInterrupts(CG_SPI_MODULE);
  //SPI1BUF = 0xAA;

  /* clear any stale data */
  SPIOperate_ReadData(CG_SPI_MODULE);
}

static uint8_t Leader_QueryByte(uint8_t outByte)
{
  /* send one byte and read back the received byte */
  return (uint8_t)SPIOperate_ReadData(CG_SPI_MODULE);
}

static void InitPinHardware(void)
{
    /*PIN_MapPinInput(TapeSensor1);
    PIN_MapPinInput(TapeSensor2);
    PIN_MapPinInput(TapeSensor3);
    PIN_MapPinInput(TapeSensor4);
    PIN_MapPinInput(TapeSensor5);
    PIN_MapPinInput(UltrasonicEcho);
    PIN_MapPinOutput(UltrasonicTrigger);*/
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
        case CMD_INIT_ORIENT:
        case CMD_END_GAME:
        case CMD_SIDE_FOUND_RIGHT:
        case CMD_SIDE_FOUND_LEFT:
            return true;

        default:
            return false;
    }
}

static void HandleCommandByte(uint8_t cmd)
{
  DB_printf("handling cmd from leader: %d\r\n", cmd);
  ES_Event_t cmdEvent;
  if (cmd != prevCmd) {
    switch (cmd)
    {
    case CMD_END_GAME: {
        DB_printf("Received game end command \r\n");
        cmdEvent.EventType = ES_END_GAME;
        break;
    }
    case CMD_TRANS_FWD: {
        DB_printf("Received forward command \r\n");
        cmdEvent.EventType = ES_TRANSLATE;
        cmdEvent.EventParam = PackTranslateParam(TRANS_FULL, DIR_FWD);
        break;
    }

      case CMD_STOP: {
        DB_printf("Received STOP command\r\n");
        //outgoingCmd = CMD_STOP;
        cmdEvent.EventType = ES_STOP;
        break;
    }

    case CMD_TRANS_BWD: {
        DB_printf("Received backwards command\r\n");
        cmdEvent.EventType = ES_TRANSLATE;
        cmdEvent.EventParam = PackTranslateParam(TRANS_FULL, DIR_REV);
        break;
    }

    case CMD_ROT_CW_90: {
        DB_printf("Received CW 90 rotation command\r\n");
        cmdEvent.EventType = ES_ROTATE;
        cmdEvent.EventParam = PackRotateParam(ROT_90, ROT_CW);
        break;
    }

    case CMD_ROT_CCW_90: {
        DB_printf("Received CCW 90 rotation command\r\n");
        cmdEvent.EventType = ES_ROTATE;
        cmdEvent.EventParam = PackRotateParam(ROT_90, ROT_CCW);
        break;
    }


    case CMD_ROT_CW_45: {
        DB_printf("Received CW 45 rotation command\r\n");
        cmdEvent.EventType = ES_ROTATE;
        cmdEvent.EventParam = PackRotateParam(ROT_45, ROT_CW);
        break;
    }

    case CMD_ROT_CCW_45: {
        DB_printf("Received CCW 45 rotation command\r\n");
        cmdEvent.EventType = ES_ROTATE;
        cmdEvent.EventParam = PackRotateParam(ROT_45, ROT_CCW);
        break;
    }

    case CMD_TAPE_T_DETECT: {
        DB_printf("Received TAPE detect command\r\n");
        cmdEvent.EventType = ES_TAPE_DETECT;
        break;
    }

    case CMD_LINE_FOLLOW: {
        DB_printf("Received LINE FOLLOW command\r\n");
        cmdEvent.EventType = ES_LINE_FOLLOW;
        break;
    }

    case CMD_GET_BEACON_FREQ: {
        DB_printf("Received GET BEACON FREQ command\r\n");
        cmdEvent.EventType = ES_BEACON_SIGNAL;
        //cmdEvent.EventType = ES_ALIGN_ULTRASONICS;
        break;
    }

    case CMD_TESTING: {
        DB_printf("Received TESTING command\r\n");
        //outgoingCmd = CMD_TESTING;
        break;
    }

    case CMD_QUERY: {
        DB_printf("Received QUERY command from SPILeaderService! Outgoing cmd %d\r\n", outgoingCmd);
        //outgoingCmd = CMD_TESTING;
        //DB_printf("Sending Testing Cmd to Leader!\r\n");
        break;
    }

    case CMD_INIT_ORIENT: {
        DB_printf("Received INIT ORIENT command from SPILeaderService!\r\n");
        cmdEvent.EventType = ES_ALIGN_ULTRASONICS;
        DB_printf("Posting align ultrasonics command to nav service!\r\n");
        break;
    }
    
    case CMD_NOOP: {
        DB_printf("Received NOOP command from SPILeaderService!\r\n");
        break;
    }

    default:
        break;

    }
    PostNavigateService(cmdEvent);
    prevCmd = cmd;
  } 
}

void __ISR(_SPI1_VECTOR, IPL7SOFT) SPI1_Handler(void) {
    //__builtin_disable_interrupts();
    ES_Event_t NewEvent;
    if (SPI1STATbits.SPIROV) {
        SPI1STATCLR = _SPI1STAT_SPIROV_MASK;
    }
    if (IFS1bits.SPI1RXIF) {
        incomingCmd = SPI1BUF;     // what leader sent
        //DB_printf("%d\r\n", incomingCmd);
        SPI1BUF = outgoingCmd;
        if (incomingCmd != curCmd) {
           NewEvent.EventType = ES_SPI_RECEIVED;
           curCmd = incomingCmd;
           NewEvent.EventParam = curCmd;
           bool test = PostSPIFollowerService(NewEvent);
           DB_printf("%d\r\n", curCmd);
           //DB_printf("%d\r\n", test);
        }
    }
    IFS1CLR = _IFS1_SPI1RXIF_MASK;
    //__builtin_enable_interrupts();
}
