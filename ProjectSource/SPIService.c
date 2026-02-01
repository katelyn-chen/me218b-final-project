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
****************************************************************************/

#include "SPIService.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"

#include "Lab8_Events.h"
#include "MotorService.h"

/*============================== CONFIG ==============================*/
/* pick an ES timer ID that is free in your ES_Configure.c */
#ifndef SPI_POLL_TIMER
#define SPI_POLL_TIMER  1u
#endif

/* how often to ask for a new command (ms) */
#define SPI_POLL_MS     50u

/* Chip Select pin (you MUST set this to match your wiring) */
#define CG_CS_LAT       LATBbits.LATB7
#define CG_CS_TRIS      TRISBbits.TRISB7
#define CG_CS_ANSEL     ANSELBbits.ANSB7

/* SPI module choice */
#define USE_SPI1        1

/*============================== STATE ==============================*/
static uint8_t MyPriority;

/*====================== PRIVATE FUNCTION PROTOS =====================*/
static void InitSPIHardware(void);
static void CG_Select(void);
static void CG_Deselect(void);
static uint8_t SPI_TxRxByte(uint8_t outByte);

static void HandleCommandByte(uint8_t cmd);

/*=========================== PUBLIC API =============================*/
bool InitSPIService(uint8_t Priority)
{
  MyPriority = Priority;

  InitSPIHardware();

  /* kick the polling timer */
  ES_Timer_InitTimer(SPI_POLL_TIMER, SPI_POLL_MS);

  dbprintf("SPIService: init done\r\n");

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

  if ((ThisEvent.EventType == ES_TIMEOUT) && (ThisEvent.EventParam == SPI_POLL_TIMER))
  {
    /* query follower for next command */
    CG_Select();
    uint8_t cmd = SPI_TxRxByte(CMD_QUERY);
    CG_Deselect();

    /* ignore “noop” if they send that */
    if (cmd != CMD_NOOP)
    {
      HandleCommandByte(cmd);
    }

    ES_Timer_InitTimer(SPI_POLL_TIMER, SPI_POLL_MS);
  }

  return ReturnEvent;
}

/*=========================== SPI HELPERS ============================*/
static void InitSPIHardware(void)
{
  /* CS pin */
  CG_CS_TRIS = 0;
  CG_CS_ANSEL = 0;
  CG_Deselect();

#if USE_SPI1
  /* Basic SPI1 master setup (8-bit) */
  SPI1CON = 0;
  SPI1BRG = 19;           /* example: sets SCK speed; adjust if needed */
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
      /* If I get a weird byte, I just print it and ignore it */
      dbprintf("SPIService: unknown cmd 0x%02X\r\n", cmd);
      break;
  }
}
