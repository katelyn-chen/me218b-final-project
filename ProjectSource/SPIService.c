/****************************************************************************
  Module
    SPIService.c  (Lab 8)

  Summary
    - Configure SPI in leader (master) mode using the SPI HAL
    - Periodically query the Command Generator
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

  Wiring (from Amanda's notes):
  @Amanda, if i messed up this please fix :) thanks!
    CommandGen SDI  -> PIC32 SDO1  (RB5,  pin 14)
    CommandGen SDO  -> PIC32 SDI1  (RB11, pin 22)
    CommandGen SCK  -> PIC32 SCK1  (RB14, pin 25)
    CommandGen SS   -> PIC32 CS    (RB15, pin ??)
****************************************************************************/

#include "SPIService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "PIC32_SPI_HAL.h"

#include "MotorService.h"

/*============================== CONFIG ==============================*/
/* use the symbolic timer name from ES_Configure.h */
#ifndef SPI_TIMER
#define SPI_TIMER            0u
#endif

/* how often to ask for a new command (ms) */
#define SPI_POLL_MS          50u

/* CS pin (manual GPIO control) */
#define CG_CS_LAT            LATBbits.LATB15
#define CG_CS_TRIS           TRISBbits.TRISB15
#define CG_CS_ANSEL          ANSELBbits.ANSB15

/* SPI module selection */
#define CG_SPI_MODULE        SPI_SPI1

/* SPI clock period (ns). 500 kHz -> 2000 ns period */
#define SPI_CLK_PERIOD_NS    2000u

/* ---------------- Command Generator bytes (Appendix A) ----------------
   keeping these here so SPIService.c is self-contained.
----------------------------------------------------------------------- */
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

/*============================== STATE ==============================*/
static uint8_t MyPriority;

/* protocol state:
   - see 0xFF marker -> next query returns actual command
*/
static bool WaitingForRealCommand = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void CG_Select(void);
static void CG_Deselect(void);
static uint8_t CG_QueryByte(uint8_t outByte);

static void HandleCommandByte(uint8_t cmd);
static bool IsKnownCommand(uint8_t cmd);

/*=========================== PUBLIC API =============================*/
bool InitSPIService(uint8_t Priority)
{
  MyPriority = Priority;

  InitSPIHardware();

  ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);

  DB_printf("SPIService: init done\r\n");

  ES_Event_t ThisEvent = { ES_INIT, 0 };
  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostSPIService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunSPIService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent = { ES_NO_EVENT, 0 };

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SPI_TIMER))
  {
    uint8_t rx;

    /* query follower for next byte */
    //CG_Select();
    //for (volatile int i = 0; i <50; i++){}
    rx = CG_QueryByte(CMD_QUERY);
    //for (volatile int i = 0; i <50; i++){}
    //CG_Deselect();

    DB_printf("SPI rx = 0x%d\r\n", rx);

    /* ignore unknown bytes (do not silently convert) */
    if (!IsKnownCommand(rx))
    {
      DB_printf("SPIService: unknown rx 0x%02X\r\n", rx);
      ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
      return ReturnEvent;
    }

    /* protocol behavior:
       - 0xFF by itself is the ?new command ready? marker
       - the NEXT query returns the actual command value
    */
    if (rx == CMD_NOOP)
    {
      WaitingForRealCommand = true;
    }
    else
    {
      if (WaitingForRealCommand)
      {
        HandleCommandByte(rx);
        WaitingForRealCommand = false;
      }
      else
      {
        HandleCommandByte(rx);
      }
    }

    ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
  }

  return ReturnEvent;
}

/*=========================== SPI HELPERS ============================*/
static void InitSPIHardware(void)
{
  /* CS pin as digital output, start deselected */
  //CG_CS_ANSEL = 0;
  //CG_CS_TRIS = 0;
  //CG_CS_LAT = 1;

  /* HAL SPI setup */
  SPISetup_BasicConfig(CG_SPI_MODULE);

  SPISetup_SetLeader(CG_SPI_MODULE, SPI_SMP_MID);
   /* map data pins */
  SPISetup_MapSDOutput(CG_SPI_MODULE, SPI_RPB5);   /* SDO1 -> RB5 */
  SPISetup_MapSDInput(CG_SPI_MODULE,  SPI_RPB11);  /* SDI1 <- RB11 */
  SPISetup_MapSSOutput(CG_SPI_MODULE, SPI_RPB15);

  /* clock mode settings (matches common CKP=0, CKE=1 style) */
  SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);    // CKP = 1
  SPISetup_SetActiveEdge(CG_SPI_MODULE, SPI_FIRST_EDGE);   // CKE = 0
  SPISetEnhancedBuffer(CG_SPI_MODULE, true);
  SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

  /* set SPI bit time (ns) */
  SPISetup_SetBitTime(CG_SPI_MODULE, SPI_CLK_PERIOD_NS);

  SPISetup_EnableSPI(CG_SPI_MODULE);

  /* clear any stale data */
  SPIOperate_ReadData(CG_SPI_MODULE);
}

static void CG_Select(void)
{
  CG_CS_LAT = 0;
}

static void CG_Deselect(void)
{
  CG_CS_LAT = 1;
}

static uint8_t CG_QueryByte(uint8_t outByte)
{
  /* send one byte and read back the received byte */
  //DB_printf("%d", outByte);
  SPIOperate_SPI1_Send8Wait(outByte);
  
  //SPI1BUF = tx;
  //while (!SPI1STATbits.SPIRBF);
  //return SPI1BUF;
  /* read data register */
  return (uint8_t)SPIOperate_ReadData(CG_SPI_MODULE);
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
  ES_Event_t e;

  switch (cmd)
  {
    case CMD_STOP:
      e.EventType = ES_STOP; e.EventParam = 0;
      PostMotorService(e);
      break;

    case CMD_ALIGN:
      e.EventType = ES_ALIGN; e.EventParam = 0;
      PostMotorService(e);
      break;

    case CMD_TAPE_DETECT:
      e.EventType = ES_TAPE_DETECT; e.EventParam = 0;
      PostMotorService(e);
      break;

    case CMD_ROT_CW_45:
      e.EventType = ES_ROTATE;
      e.EventParam = PackRotateParam(ROT_45, ROT_CW);
      PostMotorService(e);
      break;

    case CMD_ROT_CW_90:
      e.EventType = ES_ROTATE;
      e.EventParam = PackRotateParam(ROT_90, ROT_CW);
      PostMotorService(e);
      break;

    case CMD_ROT_CCW_45:
      e.EventType = ES_ROTATE;
      e.EventParam = PackRotateParam(ROT_45, ROT_CCW);
      PostMotorService(e);
      break;

    case CMD_ROT_CCW_90:
      e.EventType = ES_ROTATE;
      e.EventParam = PackRotateParam(ROT_90, ROT_CCW);
      PostMotorService(e);
      break;

    case CMD_TRANS_FWD_HALF:
      e.EventType = ES_TRANSLATE;
      e.EventParam = PackTranslateParam(TRANS_HALF, DIR_FWD);
      PostMotorService(e);
      break;

    case CMD_TRANS_FWD_FULL:
      e.EventType = ES_TRANSLATE;
      e.EventParam = PackTranslateParam(TRANS_FULL, DIR_FWD);
      PostMotorService(e);
      break;

    case CMD_TRANS_REV_HALF:
      e.EventType = ES_TRANSLATE;
      e.EventParam = PackTranslateParam(TRANS_HALF, DIR_REV);
      PostMotorService(e);
      break;

    case CMD_TRANS_REV_FULL:
      e.EventType = ES_TRANSLATE;
      e.EventParam = PackTranslateParam(TRANS_FULL, DIR_REV);
      PostMotorService(e);
      break;

    default:
      break;
  }
}
