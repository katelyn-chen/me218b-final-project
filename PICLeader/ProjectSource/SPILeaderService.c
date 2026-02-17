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
//#define CMD_QUERY                 0xAA
// NEEDS TO BE UPDATS WITH APPROPRIATE COMMANDS!!
#define CMD_GET_BEACON_FREQ       0xAA
#define CMD_GET_LINE_SENSORS      0x0A
#define CMD_MOTOR_FWD             0x01

#define CMD_QUERY                 0xAA
#define CMD_NOOP                  0xFF   /* also used as ?new cmd ready? marker */

#define CMD_STOP                  0x00

#define CMD_ROT_CW_90             0x02
#define CMD_ROT_CW_45             0x03
#define CMD_ROT_CCW_90            0x04
#define CMD_ROT_CCW_45            0x05

#define CMD_TRANS_FWD_HALF        0x08
#define CMD_TRANS_FWD_FULL        0x09
#define CMD_TRANS_REV_HALF        0x10
#define CMD_TRANS_REV_FULL        0x11

#define CMD_ALIGN                 0x20
#define CMD_TAPE_DETECT           0x40

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  SEND,
  RECEIVE,
  DEBUG
} LeaderState_t;

/*---------------------------- Module Variables --------------------------*/
static uint8_t MyPriority;
LeaderState_t curState;
static uint8_t curCmd;
uint8_t prevCmd;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static uint8_t Follower_QueryByte(uint8_t outByte);
static void HandleCommandByte(uint8_t cmd);
static bool IsKnownCommand(uint8_t cmd);

/*------------------------------ Module Code ------------------------------*/
bool InitSPILeaderService(uint8_t Priority)
{
  MyPriority = Priority;
  curState = SEND;
  InitSPIHardware();
  ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
  DB_printf("SPILeaderService initialized\r\n");

  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_INIT;
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostSPILeaderService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPILeaderService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SPI_TIMER))
  {
    switch (curState) {
        case SEND:
        {
            curCmd = CMD_MOTOR_FWD;
            DB_printf("Leader Sending command! %d\n", curCmd);
            SPIOperate_SPI1_Send8Wait(curCmd);
            ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
            break;
        }
        
        case RECEIVE:
        {
            uint8_t followerData;
            followerData = Follower_QueryByte(CMD_QUERY);

            /* ignore unknown bytes (do not silently convert) */
            if (!IsKnownCommand(followerData))
            {
                DB_printf("SPIService: unknown cmd 0x%02X\r\n", followerData);
                ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
                return ReturnEvent;
            }

            //HandleCommandByte(followerData);

            DB_printf("Leader received byte: %d\n", followerData);
            break;
        }
    }
    ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
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
    case CMD_NOOP:
    case CMD_STOP:
    case CMD_ROT_CW_90:
    case CMD_ROT_CW_45:
    case CMD_ROT_CCW_90:
    case CMD_ROT_CCW_45:
    case CMD_TRANS_FWD_HALF:
    case CMD_TRANS_FWD_FULL:
    case CMD_TRANS_REV_HALF:
    case CMD_TRANS_REV_FULL:
    case CMD_ALIGN:
    case CMD_TAPE_DETECT:
    case CMD_MOTOR_FWD:
    return true;

    default:
      return false;
  }
}

/*static void HandleCommandByte(uint8_t cmd)
{
  ES_Event_t e;
  //DB_printf("SPIService posting cmd=%d\r\n", cmd);
  if (cmd != prevCmd) {
    switch (cmd)
    {
      case CMD_STOP:
        DB_printf("Posting stop from SPIService \r\n");
        e.EventType = ES_STOP; e.EventParam = 0;
        PostMotorService(e);
        break;

      case CMD_ALIGN:
        DB_printf("SPI align %d\n", cmd);
        e.EventType = ES_ALIGN; e.EventParam = 0;
        PostMotorService(e);
        break;

      case CMD_TAPE_DETECT:
        DB_printf("SPI tape %d\n", cmd);
        e.EventType = ES_TAPE_DETECT; e.EventParam = 0;
        PostMotorService(e);
        break;

      case CMD_ROT_CW_45:
        DB_printf("SPI CW 45 %d/n", cmd);
        e.EventType = ES_ROTATE;
        e.EventParam = PackRotateParam(ROT_45, ROT_CW);
        PostMotorService(e);
        break;

      case CMD_ROT_CW_90:
        DB_printf("SPI CW 90 %d\n", cmd);
        e.EventType = ES_ROTATE;
        e.EventParam = PackRotateParam(ROT_90, ROT_CW);
        PostMotorService(e);
        break;

      case CMD_ROT_CCW_45:
        DB_printf("SPI CCW 45 %d\n", cmd);
        e.EventType = ES_ROTATE;
        e.EventParam = PackRotateParam(ROT_45, ROT_CCW);
        PostMotorService(e);
        break;

      case CMD_ROT_CCW_90:
        DB_printf("SPI CCW 90 %d\n", cmd);
        e.EventType = ES_ROTATE;
        e.EventParam = PackRotateParam(ROT_90, ROT_CCW);
        PostMotorService(e);
        break;

      case CMD_TRANS_FWD_HALF:
        DB_printf("SPI fwd half %d\n", cmd);
        e.EventType = ES_TRANSLATE;
        e.EventParam = PackTranslateParam(TRANS_HALF, DIR_FWD);
        PostMotorService(e);
        break;

      case CMD_TRANS_FWD_FULL:
        DB_printf("SPI fwd full %d\n", cmd);
        e.EventType = ES_TRANSLATE;
        e.EventParam = PackTranslateParam(TRANS_FULL, DIR_FWD);
        PostMotorService(e);
        break;

      case CMD_TRANS_REV_HALF:
        DB_printf("SPI rev half %d\n", cmd);
        e.EventType = ES_TRANSLATE;
        e.EventParam = PackTranslateParam(TRANS_HALF, DIR_REV);
        PostMotorService(e);
        break;

      case CMD_TRANS_REV_FULL:
        DB_printf("SPI rev half %d\n", cmd);
        e.EventType = ES_TRANSLATE;
        e.EventParam = PackTranslateParam(TRANS_FULL, DIR_REV);
        PostMotorService(e);
        break;

      default:
        break;
        
    }
    prevCmd = cmd;
  } 
 
}*/
