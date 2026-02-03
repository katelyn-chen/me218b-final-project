/****************************************************************************
  Module
    SPIService.c  (Lab 8)

  Summary
    - Configure SPI in leader (master) mode
    - Periodically query the Command Generator
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

  Notes I’m using:
    - Command Generator is the follower (slave)
    - My PIC is the leader (master)
    - I poll on a timer (simple and easy to debug)
    - Protocol detail:
        * Send 0xAA to query
        * When a NEW command is ready, the generator returns one 0xFF,
          then the next query returns the actual command byte
****************************************************************************/

#include "SPIService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "MotorService.h"

/*============================== CONFIG ==============================*/
/* use the symbolic timer names from ES_Configure.h */
#ifndef SPI_TIMER
#define SPI_TIMER 0
#endif

/* how often to ask for a new command (ms) */
#define SPI_POLL_MS     50u

/* Chip Select pin (you MUST set this to match your wiring) */
#define CG_CS_LAT       LATBbits.LATB7
#define CG_CS_TRIS      TRISBbits.TRISB7
//#define CG_CS_ANSEL     ANSELBbits.ansB7

/* SPI module choice */
#define USE_SPI1        1

/* ---------------- Command Generator bytes (Appendix A) ----------------
   I’m keeping these here so SPIService.c is self-contained.
   Update only if the Command Generator doc changes.
----------------------------------------------------------------------- */
#define CMD_QUERY                 0xAA
#define CMD_NOOP                  0xFF   /* also used as “new cmd ready” marker */

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

/* protocol state: when I see a single 0xFF, next query gives the real new cmd */
static bool ExpectNewCommand = false;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void CG_Select(void);
static void CG_Deselect(void);
static uint8_t SPI_TxRxByte(uint8_t outByte);

static void HandleCommandByte(uint8_t cmd);
static bool IsKnownCommand(uint8_t cmd);

/*=========================== PUBLIC API =============================*/
bool InitSPIService(uint8_t Priority)
{
  MyPriority = Priority;

  InitSPIHardware();

  /* kick the polling timer */
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
    /* query follower for next byte */
    CG_Select();
    uint8_t rx = SPI_TxRxByte(CMD_QUERY);
    CG_Deselect();

    /* treat unknown bytes as 0xFF per spec */
    if (!IsKnownCommand(rx))
    {
      rx = CMD_NOOP;
    }

    /* protocol behavior:
       - 0xFF can mean “not ready yet” OR “new cmd ready marker”
       - the doc says: whenever a NEW command is ready, it returns a single 0xFF,
         and the NEXT query returns the new command value
    */
    if (rx == CMD_NOOP)
    {
      /* if we were waiting for a new command, and got 0xFF again,
         just keep waiting */
      ExpectNewCommand = true;
    }
    else
    {
      /* we received a real command byte */
      if (ExpectNewCommand)
      {
        /* this is the actual “new command” value after the single 0xFF marker */
        HandleCommandByte(rx);
        ExpectNewCommand = false;
      }
      else
      {
        /* even if it’s not “new”, the generator can keep returning the same
           command byte; I still handle it because it keeps behavior consistent */
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
  /* CS pin */
  CG_CS_TRIS = 0;
//  CG_CS_ANSEL = 0;
  CG_Deselect();

#if USE_SPI1
  /* Basic SPI1 master setup (8-bit) */
  SPI1CON = 0;
  SPI1BRG = 19;           /* sets SCK speed; adjust if needed */
  SPI1STATbits.SPIROV = 0;
  SPI1CONbits.MSTEN = 1;  /* master */
  SPI1CONbits.CKP = 0;
  SPI1CONbits.CKE = 1;
  SPI1CONbits.SMP = 0;
  SPI1CONbits.MODE16 = 0;
  SPI1CONbits.MODE32 = 0;
  SPI1CONbits.ON = 1;
#else
  /* If you use SPI2, duplicate the same idea with SPI2 registers */
#endif
}

static void CG_Select(void)
{
  CG_CS_LAT = 0;
}

static void CG_Deselect(void)
{
  CG_CS_LAT = 1;
}

static uint8_t SPI_TxRxByte(uint8_t outByte)
{
#if USE_SPI1
  /* clear overflow just in case */
  SPI1STATbits.SPIROV = 0;

  /* write starts the transfer */
  SPI1BUF = outByte;

  /* wait until receive is done */
  while (!SPI1STATbits.SPIRBF) { }

  return (uint8_t)SPI1BUF;
#else
  return 0;
#endif
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
      /* unknown bytes are treated like 0xFF anyway, so I just ignore here */
      break;
  }
}
