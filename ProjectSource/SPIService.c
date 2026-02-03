/****************************************************************************
  Module
    SPIService.c  (Lab 8)

  Summary
    - Configure SPI1 in leader (master) mode using SPI HAL
    - Periodically query the Command Generator
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

  Wiring (from lab notes):
    CommandGen SDI  -> PIC32 SDO1  (RB5,  pin 14)
    CommandGen SDO  -> PIC32 SDI1  (RB11, pin 22)
    CommandGen SCK  -> PIC32 SCK1  (RB14, pin 25)
    CommandGen SS   -> PIC32 SS1   (RB15, pin 26)
****************************************************************************/

#include "SPIService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "MotorService.h"
#include "PIC32_SPI_HAL.h"

/*============================== CONFIG ==============================*/
#ifndef SPI_TIMER
#define SPI_TIMER               0u
#endif

#define SPI_POLL_MS             50u

/* Command Generator protocol bytes */
#define CMD_QUERY               0xAA
#define CMD_NOOP                0xFF

#define CMD_STOP                0x00
#define CMD_ROT_CW_90           0x02
#define CMD_ROT_CW_45           0x03
#define CMD_ROT_CCW_90          0x04
#define CMD_ROT_CCW_45          0x05

#define CMD_TRANS_FWD_HALF      0x08
#define CMD_TRANS_FWD_FULL      0x09
#define CMD_TRANS_REV_HALF      0x10
#define CMD_TRANS_REV_FULL      0x11

#define CMD_ALIGN               0x20
#define CMD_TAPE_DETECT         0x40

/* SPI mode parameters */
#define SPI_IDLE_STATE          SPI_CLK_HI
#define SPI_ACTIVE_EDGE         SPI_SECOND_EDGE
#define SPI_SAMPLE_PHASE        SPI_SMP_MID
#define SPI_BITTIME_NS          2000u   /* 500 kHz */

/* SPI1 pin mapping */
#define SPI1_SS_PIN             SPI_RPB15
#define SPI1_SDO_PIN            SPI_RPB5
#define SPI1_SDI_PIN            SPI_RPB11

/*============================== STATE ==============================*/
static uint8_t MyPriority;
static bool WaitingForRealCommand = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static bool IsKnownCommand(uint8_t cmd);
static void HandleCommandByte(uint8_t cmd);

/*=========================== PUBLIC API =============================*/
bool InitSPIService(uint8_t Priority)
{
  MyPriority = Priority;

  InitSPIHardware();

  ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);

  DB_printf("SPIService initialized (HAL, SS=RB15)\r\n");

  ES_Event_t InitEvent = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, InitEvent);
}

bool PostSPIService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPIService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  if ((ThisEvent.EventType == ES_TIMEOUT) &&
      (ThisEvent.EventParam == SPI_TIMER))
  {
    /* Send query byte and wait for SS cycle to complete */
    SPIOperate_SPI1_Send8Wait(CMD_QUERY);

    /* Read received byte */
    uint8_t rx = (uint8_t)SPIOperate_ReadData(SPI_SPI1);

    static uint16_t PrintDivider = 0;
    if ((PrintDivider++ % 10u) == 0u)
    {
      DB_printf("SPI rx (dec) = %u\r\n", (unsigned int)rx);
    }

    if (!IsKnownCommand(rx))
    {
      DB_printf("SPI unknown byte: 0x%02X\r\n", rx);
      ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
      return ReturnEvent;
    }

    if (rx == CMD_NOOP)
    {
      WaitingForRealCommand = true;
    }
    else
    {
      HandleCommandByte(rx);
      WaitingForRealCommand = false;
    }

    ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
  }

  return ReturnEvent;
}

/*=========================== SPI HAL SETUP ============================*/
static void InitSPIHardware(void)
{
  bool Success = true;

  Success &= SPISetup_BasicConfig(SPI_SPI1);
  Success &= SPISetup_SetLeader(SPI_SPI1, SPI_SAMPLE_PHASE);

  Success &= SPISetup_MapSSOutput(SPI_SPI1, SPI1_SS_PIN);
  Success &= SPISetup_MapSDOutput(SPI_SPI1, SPI1_SDO_PIN);
  Success &= SPISetup_MapSDInput (SPI_SPI1, SPI1_SDI_PIN);

  Success &= SPISetup_SetClockIdleState(SPI_SPI1, SPI_IDLE_STATE);
  Success &= SPISetup_SetActiveEdge(SPI_SPI1, SPI_ACTIVE_EDGE);

  Success &= SPISetup_SetXferWidth(SPI_SPI1, SPI_8BIT);
  Success &= SPISetEnhancedBuffer(SPI_SPI1, false);

  Success &= SPISetup_SetBitTime(SPI_SPI1, SPI_BITTIME_NS);
  Success &= SPISetup_EnableSPI(SPI_SPI1);

  if (!Success)
  {
    DB_printf("SPI HAL configuration failed\r\n");
  }
  else
  {
    DB_printf("SPI HAL configuration successful\r\n");
  }
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
      return true;
    default:
      return false;
  }
}

static void HandleCommandByte(uint8_t cmd)
{
  ES_Event_t Event;

  switch (cmd)
  {
    case CMD_STOP:
      Event.EventType = ES_STOP;
      Event.EventParam = 0;
      break;

    case CMD_ALIGN:
      Event.EventType = ES_ALIGN;
      Event.EventParam = 0;
      break;

    case CMD_TAPE_DETECT:
      Event.EventType = ES_TAPE_DETECT;
      Event.EventParam = 0;
      break;

    case CMD_ROT_CW_45:
      Event.EventType = ES_ROTATE;
      Event.EventParam = PackRotateParam(ROT_45, ROT_CW);
      break;

    case CMD_ROT_CW_90:
      Event.EventType = ES_ROTATE;
      Event.EventParam = PackRotateParam(ROT_90, ROT_CW);
      break;

    case CMD_ROT_CCW_45:
      Event.EventType = ES_ROTATE;
      Event.EventParam = PackRotateParam(ROT_45, ROT_CCW);
      break;

    case CMD_ROT_CCW_90:
      Event.EventType = ES_ROTATE;
      Event.EventParam = PackRotateParam(ROT_90, ROT_CCW);
      break;

    case CMD_TRANS_FWD_HALF:
      Event.EventType = ES_TRANSLATE;
      Event.EventParam = PackTranslateParam(TRANS_HALF, DIR_FWD);
      break;

    case CMD_TRANS_FWD_FULL:
      Event.EventType = ES_TRANSLATE;
      Event.EventParam = PackTranslateParam(TRANS_FULL, DIR_FWD);
      break;

    case CMD_TRANS_REV_HALF:
      Event.EventType = ES_TRANSLATE;
      Event.EventParam = PackTranslateParam(TRANS_HALF, DIR_REV);
      break;

    case CMD_TRANS_REV_FULL:
      Event.EventType = ES_TRANSLATE;
      Event.EventParam = PackTranslateParam(TRANS_FULL, DIR_REV);
      break;

    default:
      return;
  }

  PostMotorService(Event);
}
