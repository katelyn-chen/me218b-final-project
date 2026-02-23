/****************************************************************************
  Module
    SPIFollowerService.c

  Summary
    - Configure SPI in follower (slave) mode using the SPI HAL
    - Respond to leader queries with a single outgoing status byte
    - Convert leader command bytes into events for NavigateService
    - Use outgoing status bytes to report sensor/nav results back to leader

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

/* SPI module selection */
#define CG_SPI_MODULE        SPI_SPI1

/* SPI clock period (ns). 500 kHz -> 2000 ns period */
//#define SPI_CLK_PERIOD_NS    2000u
#define SPI_CLK_PERIOD_NS    50000u

/* ---------------- Command Generator bytes (Appendix A) ----------------
   keeping these here so SPIFollowerService.c is self-contained.
----------------------------------------------------------------------- */

/* leader -> follower motion commands */
#define CMD_STOP                  0x00
#define CMD_TRANS_FWD             0x01
#define CMD_TRANS_BWD             0x02
#define CMD_ROT_CW_90             0x03
#define CMD_ROT_CCW_90            0x04
#define CMD_ROT_CCW_45            0x05
#define CMD_ROT_CW_45             0x06
#define CMD_TAPE_T_DETECT         0x07
#define CMD_LINE_FOLLOW           0x08

/* leader -> follower beacon/nav commands */
#define CMD_GET_BEACON_FREQ       0x11
#define CMD_INIT_ORIENT           0x12

/* follower -> leader status bytes */
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

/*
  collect/dispense sync (leader controls servos, follower freezes motors)
  these are optional "do nothing except stop" commands so the leader can
  keep the follower from drifting while servos run.
*/
#define CMD_COLLECT_START         0x20
#define CMD_DISPENSE_START        0x21

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  RECEIVE = 0
} FollowerState_t;

typedef enum {
  SIDE_RIGHT = 0,
  SIDE_LEFT  = 1
} SideIndicate_t;

/*---------------------------- Module Variables --------------------------*/
static uint8_t MyPriority;
static FollowerState_t curState;

static volatile uint8_t incomingCmd;
static uint8_t curCmd;
static uint8_t prevCmd;

/* outgoing status byte returned to leader on CMD_QUERY */
static volatile uint8_t outgoingCmd;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void InitPinHardware(void);

static void HandleCommandByte(uint8_t cmd);
static bool IsKnownLeaderCommand(uint8_t cmd);

static void SetOutgoingOnce(uint8_t statusByte);

/*------------------------------ Module Code ------------------------------*/
bool InitSPIFollowerService(uint8_t Priority)
{
  MyPriority = Priority;
  curState = RECEIVE;

  incomingCmd = CMD_NOOP;
  curCmd      = CMD_NOOP;
  prevCmd     = CMD_NOOP;
  outgoingCmd = CMD_NOOP;

  InitSPIHardware();
  InitPinHardware();

  DB_printf("SPIFollowerService initialized\r\n");

  ES_Event_t ThisEvent;
  ThisEvent.EventType = ES_INIT;
  ThisEvent.EventParam = 0;
  ES_PostToService(MyPriority, ThisEvent);

  return true;
}

bool PostSPIFollowerService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  switch (ThisEvent.EventType)
  {
    case ES_SIDE_INDICATED:
    {
      /* side event param matches nav: RIGHT=0, LEFT=1 in current codebase */
      if (ThisEvent.EventParam == SIDE_RIGHT)
      {
        SetOutgoingOnce(CMD_SIDE_FOUND_RIGHT);
      }
      else
      {
        SetOutgoingOnce(CMD_SIDE_FOUND_LEFT);
      }
      break;
    }

    case ES_BEACON_FOUND:
    {
      /*
        BeaconService posts ES_BEACON_FOUND with packed param:
          [15:8] id, [7:0] side
        Only the id is needed for leader debug / alignment.
      */
      BeaconId_t id;
      BeaconSide_t side;
      UnpackBeaconParam((uint16_t)ThisEvent.EventParam, &id, &side);
      (void)side;

      DB_printf("Beacon found! ID: %u\r\n", (unsigned)id);

      switch (id)
      {
        case BEACON_ID_G: SetOutgoingOnce(CMD_BEACON_G_FOUND); break;
        case BEACON_ID_B: SetOutgoingOnce(CMD_BEACON_B_FOUND); break;
        case BEACON_ID_R: SetOutgoingOnce(CMD_BEACON_R_FOUND); break;
        case BEACON_ID_L: SetOutgoingOnce(CMD_BEACON_L_FOUND); break;
        default: break;
      }
      break;
    }

    case ES_SPI_RECEIVED:
    {
      uint8_t cmd = (uint8_t)ThisEvent.EventParam;
      DB_printf("Received byte from leader %u\r\n", (unsigned)cmd);

      /* ignore unknown bytes */
      if (!IsKnownLeaderCommand(cmd))
      {
        DB_printf("SPIFollowerService: unknown leaderCmd 0x%02X\r\n", cmd);
        break;
      }

      HandleCommandByte(cmd);
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
  /*
    Wiring
      MOSI: Leader SDO (RB5, pin 14) --> Follower SDI (RA0, pin 2)
      MISO: Leader SDI (RB8, pin 17) --> Follower SDO (RA3, pin 10)
      Leader SCK (RB14, pin 25) --> Follower SCK1  (RB14, pin 25)
      Leader SS (RB15, pin 26) --> Follower CS (RB15, pin 26)
  */

  DB_printf("Follower SPI Setup\r\n");

  SPISetup_BasicConfig(CG_SPI_MODULE);
  SPISetup_SetFollower(CG_SPI_MODULE);

  /* clock pin analog off */
  ANSELBbits.ANSB14 = 0;

  /* map data pins */
  SPISetup_MapSDOutput(CG_SPI_MODULE, SDOPin);
  SPISetup_MapSDInput(CG_SPI_MODULE,  SDIPin);
  SPISetup_MapSSInput(CG_SPI_MODULE,  SSPin);

  /* clock mode settings */
  SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);    // CKP = 1
  SPISetup_SetActiveEdge(CG_SPI_MODULE,    SPI_SECOND_EDGE);// CKE = 0

  SPISetEnhancedBuffer(CG_SPI_MODULE, false);
  SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

  SPISetup_EnableSPI(CG_SPI_MODULE);

  SPISetup_ConfigureInterrupts(CG_SPI_MODULE);

  /* clear any stale data */
  SPIOperate_ReadData(CG_SPI_MODULE);

  /* prime the first response byte */
  SPI1BUF = CMD_NOOP;
}

static void InitPinHardware(void)
{
  PIN_MapPinInput(TapeSensor1);
  PIN_MapPinInput(TapeSensor2);
  PIN_MapPinInput(TapeSensor3);
  PIN_MapPinInput(TapeSensor4);
  PIN_MapPinInput(TapeSensor5);

  PIN_MapPinInput(UltrasonicEcho);
  PIN_MapPinOutput(UltrasonicTrigger);
}

/*======================= COMMAND -> EVENT MAP =======================*/
static bool IsKnownLeaderCommand(uint8_t cmd)
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
    case CMD_INIT_ORIENT:
    case CMD_END_GAME:
    case CMD_QUERY:
    case CMD_NOOP:
    case CMD_TESTING:
    case CMD_COLLECT_START:
    case CMD_DISPENSE_START:
      return true;

    default:
      return false;
  }
}

static void HandleCommandByte(uint8_t cmd)
{
  ES_Event_t cmdEvent = { ES_NO_EVENT, 0 };

  if (cmd == prevCmd)
  {
    return;
  }

  DB_printf("handling cmd from leader: %u\r\n", (unsigned)cmd);

  switch (cmd)
  {
    case CMD_END_GAME:
      DB_printf("Received END GAME command\r\n");
      cmdEvent.EventType = ES_END_GAME;
      break;

    case CMD_STOP:
      DB_printf("Received STOP command\r\n");
      cmdEvent.EventType = ES_STOP;
      break;

    case CMD_TRANS_FWD:
      DB_printf("Received TRANS FWD command\r\n");
      cmdEvent.EventType  = ES_TRANSLATE;
      cmdEvent.EventParam = PackTranslateParam(TRANS_FULL, DIR_FWD);
      break;

    case CMD_TRANS_BWD:
      DB_printf("Received TRANS BWD command\r\n");
      cmdEvent.EventType  = ES_TRANSLATE;
      cmdEvent.EventParam = PackTranslateParam(TRANS_FULL, DIR_REV);
      break;

    case CMD_ROT_CW_90:
      DB_printf("Received ROT CW 90 command\r\n");
      cmdEvent.EventType  = ES_ROTATE;
      cmdEvent.EventParam = PackRotateParam(ROT_90, ROT_CW);
      break;

    case CMD_ROT_CCW_90:
      DB_printf("Received ROT CCW 90 command\r\n");
      cmdEvent.EventType  = ES_ROTATE;
      cmdEvent.EventParam = PackRotateParam(ROT_90, ROT_CCW);
      break;

    case CMD_ROT_CW_45:
      DB_printf("Received ROT CW 45 command\r\n");
      cmdEvent.EventType  = ES_ROTATE;
      cmdEvent.EventParam = PackRotateParam(ROT_45, ROT_CW);
      break;

    case CMD_ROT_CCW_45:
      DB_printf("Received ROT CCW 45 command\r\n");
      cmdEvent.EventType  = ES_ROTATE;
      cmdEvent.EventParam = PackRotateParam(ROT_45, ROT_CCW);
      break;

    case CMD_TAPE_T_DETECT:
      DB_printf("Received TAPE T DETECT command\r\n");
      cmdEvent.EventType = ES_TAPE_DETECT;
      break;

    case CMD_LINE_FOLLOW:
      DB_printf("Received LINE FOLLOW command\r\n");
      cmdEvent.EventType = ES_LINE_FOLLOW;
      break;

    case CMD_GET_BEACON_FREQ:
      DB_printf("Received GET BEACON FREQ command\r\n");
      cmdEvent.EventType = ES_BEACON_SIGNAL;
      break;

    case CMD_INIT_ORIENT:
      DB_printf("Received INIT ORIENT command\r\n");
      cmdEvent.EventType = ES_ALIGN_ULTRASONICS;
      break;

    case CMD_COLLECT_START:
      /*
        leader starts servo-based collect sequence on PIC1
        follower stops motors to avoid drifting while the arm cycles
      */
      DB_printf("Received COLLECT START command\r\n");
      cmdEvent.EventType = ES_STOP;
      break;

    case CMD_DISPENSE_START:
      /*
        leader starts servo-based dispense sequence on PIC1
        follower stops motors during dumping
      */
      DB_printf("Received DISPENSE START command\r\n");
      cmdEvent.EventType = ES_STOP;
      break;

    case CMD_QUERY:
      /* query is handled in ISR by returning outgoingCmd */
      DB_printf("Received QUERY command\r\n");
      break;

    case CMD_NOOP:
      DB_printf("Received NOOP command\r\n");
      break;

    case CMD_TESTING:
      DB_printf("Received TESTING command\r\n");
      break;

    default:
      break;
  }

  /* NavigateService owns the drivetrain logic */
  if (cmdEvent.EventType != ES_NO_EVENT)
  {
    PostNavigateService(cmdEvent);
  }

  prevCmd = cmd;
}

/*=========================== OUTGOING STATUS ============================*/
static void SetOutgoingOnce(uint8_t statusByte)
{
  /*
    outgoingCmd is returned on the next CMD_QUERY
    leaving it latched until the query avoids missing events
  */
  outgoingCmd = statusByte;
}

/*============================== ISR ==============================*/
void __ISR(_SPI1_VECTOR, IPL7SOFT) SPI1_Handler(void)
{
  ES_Event_t NewEvent = { ES_NO_EVENT, 0 };

  if (SPI1STATbits.SPIROV)
  {
    SPI1STATCLR = _SPI1STAT_SPIROV_MASK;
  }

  if (IFS1bits.SPI1RXIF)
  {
    incomingCmd = (uint8_t)SPI1BUF;   /* leader -> follower byte */

    /*
      follower returns outgoingCmd immediately on the same transfer
      outgoingCmd gets cleared only after a CMD_QUERY so it behaves like
      a one-shot "status latch".
    */
    SPI1BUF = outgoingCmd;

    if (incomingCmd == CMD_QUERY)
    {
      outgoingCmd = CMD_NOOP;
    }

    if (incomingCmd != curCmd)
    {
      NewEvent.EventType  = ES_SPI_RECEIVED;
      curCmd              = incomingCmd;
      NewEvent.EventParam = curCmd;
      PostSPIFollowerService(NewEvent);
    }
  }

  IFS1CLR = _IFS1_SPI1RXIF_MASK;
}