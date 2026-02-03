/****************************************************************************
  Module
    SPIService.c  (Lab 8)

  Summary
    - Configure SPI in leader (master) mode
    - Periodically query the Command Generator
    - Read a command byte back
    - Convert that byte into a motion event for MotorService

  Wiring (from Amanda's notes):
  @Amanda, if i messed up this please fix :) thanks!
    CommandGen SDI  -> PIC32 SDO1  (RB5,  pin 14)
    CommandGen SDO  -> PIC32 SDI1  (RB11, pin 22)
    CommandGen SCK  -> PIC32 SCK1  (RB14, pin 25)
    CommandGen SS   -> PIC32 CS    (RB13, pin 24)
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
/* use the symbolic timer name from ES_Configure.h */
#ifndef SPI_TIMER
#define SPI_TIMER            0u
#endif

/* how often to ask for a new command (ms) */
#define SPI_POLL_MS          50u

#define PBCLK_HZ             20000000u

/* Chip Select pin (RB13, pin 24) */
#define CG_CS_LAT            LATBbits.LATB13
#define CG_CS_TRIS           TRISBbits.TRISB13
#define CG_CS_ANSEL          ANSELBbits.ANSB13

/* SPI pins (my wiring) */
#define SPI_SDO_TRIS         TRISBbits.TRISB5     /* RB5  */
// no ansel for RB5

#define SPI_SDI_TRIS         TRISBbits.TRISB11    /* RB11 */
// no ansel for RB11

#define SPI_SCK_TRIS         TRISBbits.TRISB14    /* RB14 */
#define SPI_SCK_ANSEL        ANSELBbits.ANSB14

/* ---------------- Command Generator bytes (Appendix A) ----------------
   I?m keeping these here so SPIService.c is self-contained.
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

/* SPI speed: Fsck = PBCLK / (2*(BRG+1))  */
#define SPI_BRG_VALUE             19u    /* 20MHz / (2*(19+1)) = 500kHz */

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
static uint8_t SPI_TxRxByte(uint8_t outByte);

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
    /* query follower for next byte */
    CG_Select();
    uint8_t rx = SPI_TxRxByte(CMD_QUERY);
    DB_printf("PORTB11=%d\r\n", PORTBbits.RB11); // check if it is always 0 or not
    DB_printf("rx = %d", (uint32_t) rx); // debugging
    CG_Deselect();

    /* if I get a weird byte, I print it and ignore it (don?t silently convert) */
    if (!IsKnownCommand(rx))
    {
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
      /* got a real command byte */
      if (WaitingForRealCommand)
      {
        HandleCommandByte(rx);
        WaitingForRealCommand = false;
      }
      else
      {
        /* sometimes generator repeats same command; still OK to handle */
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
  /* make sure my SPI pins are digital */
  //SPI_SDO_ANSEL = 0; no ansel for these pins
 // SPI_SDI_ANSEL = 0; no ansel for these pins
  SPI_SCK_ANSEL = 0;

  /* directions (SPI module will drive SDO/SCK, SDI is input) */
  SPI_SDO_TRIS = 0;
  SPI_SDI_TRIS = 1;
  SPI_SCK_TRIS = 0;

  /* CS pin: set high first to avoid a glitch */
  CG_CS_ANSEL = 0;
  CG_CS_LAT = 1;
  CG_CS_TRIS = 0;

  /* ---------------- PPS mapping (SPI1) ----------------
     This is where SDI/SDO are ?configured?.
     Once PPS is set, I never read/write SDI/SDO as GPIO again.

     If your specific PIC32 header uses a different SDI1R form/value,
     this is the only line you should need to tweak.
  ------------------------------------------------------*/

  /* unlock PPS */
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 0;

  /* SDO1 -> RB5
     common PPS output function code for SDO1 is 0b0011 on many ME218 PIC32 setups
  */
  RPB5Rbits.RPB5R = 0b0011;

  /* SDI1 <- RB11
     many ME218 headers use SDI1Rbits.SDI1R = 0b0011 for RPB11.
     If yours differs, change this constant to the RB11 select value for your part.
  */
  SDI1Rbits.SDI1R = 0b0011;

  /* SCK1 on RB14 is often a fixed-function SPI clock pin on these boards.
     If your part requires PPS for SCK1, you would map it here similarly.
  */

  /* lock PPS */
  CFGCONbits.IOLOCK = 1;
  SYSKEY = 0x00000000;

  /* ---------------- SPI1 setup ---------------- */
  SPI1CON = 0;
  SPI1STATbits.SPIROV = 0;

  SPI1BRG = SPI_BRG_VALUE;

  SPI1CONbits.MSTEN = 1;   /* master */
  SPI1CONbits.CKP = 0;     /* idle clock low */
  SPI1CONbits.CKE = 1;     /* data changes on falling edge, sample on rising */
  SPI1CONbits.SMP = 0;

  SPI1CONbits.MODE16 = 0;
  SPI1CONbits.MODE32 = 0;

  /* clear any garbage */
  (void)SPI1BUF;

  SPI1CONbits.ON = 1;
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
  /* clear overflow just in case */
  SPI1STATbits.SPIROV = 0;

  SPI1BUF = outByte;

  while (!SPI1STATbits.SPIRBF) { }

  return (uint8_t)SPI1BUF;
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
