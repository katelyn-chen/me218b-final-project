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
<<<<<<< HEAD
    CommandGen SS   -> PIC32 SS1   (RB15, pin 26)
=======
    CommandGen SS   -> PIC32 CS    (RB13, pin 24)
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97
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

<<<<<<< HEAD
#define CMD_TRANS_FWD_HALF      0x08
#define CMD_TRANS_FWD_FULL      0x09
#define CMD_TRANS_REV_HALF      0x10
#define CMD_TRANS_REV_FULL      0x11

#define CMD_ALIGN               0x20
#define CMD_TAPE_DETECT         0x40
=======
/* SPI pins (my wiring) */
#define SPI_SDO_TRIS         TRISBbits.TRISB5     /* RB5  */
/* no ANSEL for RB5 on many boards */

#define SPI_SDI_TRIS         TRISBbits.TRISB11    /* RB11 */
/* no ANSEL for RB11 on many boards */
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97

/* SPI mode parameters */
#define SPI_IDLE_STATE          SPI_CLK_HI
#define SPI_ACTIVE_EDGE         SPI_SECOND_EDGE
#define SPI_SAMPLE_PHASE        SPI_SMP_MID
#define SPI_BITTIME_NS          2000u   /* 500 kHz */

<<<<<<< HEAD
/* SPI1 pin mapping */
#define SPI1_SS_PIN             SPI_RPB15
#define SPI1_SDO_PIN            SPI_RPB5
#define SPI1_SDI_PIN            SPI_RPB11
=======
/* ---------------- Command Generator bytes (Appendix A) ----------------
   I’m keeping these here so SPIService.c is self-contained.
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

/* SPI speed: Fsck = PBCLK / (2*(BRG+1))  */
#define SPI_BRG_VALUE             19u    /* 20MHz / (2*(19+1)) = 500kHz */
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97

/* Optional: if your command generator returns the response on the NEXT byte,
   we must do a 2-byte transaction (request, then dummy clock). */
#define USE_TWO_BYTE_QUERY        1u
#define DUMMY_TX_BYTE             0x00

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
<<<<<<< HEAD
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
=======
    /* query follower for next byte */

    uint8_t rx = CMD_NOOP;

    CG_Select();

#if (USE_TWO_BYTE_QUERY == 1u)
    /* Many slaves only output the “answer” on the NEXT byte (because SPI is shift-based).
       So: send 0xAA to request, then send a dummy byte to clock the response out. */
    (void)SPI_TxRxByte(CMD_QUERY);
    rx = SPI_TxRxByte(DUMMY_TX_BYTE);
#else
    /* If your slave truly returns on the same byte, use this. */
    rx = SPI_TxRxByte(CMD_QUERY);
#endif

    CG_Deselect();

    /* Debug: print what we actually received (hex is much clearer) */
    DB_printf("SPI rx = 0x%02X\r\n", rx);

    /* if I get a weird byte, I print it and ignore it (don’t silently convert) */
    if (!IsKnownCommand(rx))
    {
      DB_printf("SPIService: unknown rx 0x%02X\r\n", rx);
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97
      ES_Timer_InitTimer(SPI_TIMER, SPI_POLL_MS);
      return ReturnEvent;
    }

<<<<<<< HEAD
=======
    /* protocol behavior:
       - 0xFF by itself is the “new command ready” marker
       - the NEXT query returns the actual command value
    */
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97
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
<<<<<<< HEAD
  bool Success = true;

  Success &= SPISetup_BasicConfig(SPI_SPI1);
  Success &= SPISetup_SetLeader(SPI_SPI1, SPI_SAMPLE_PHASE);
=======
  /* make sure my SPI pins are digital */
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
     This is where SDI/SDO are “configured”.
     Once PPS is set, I never read/write SDI/SDO as GPIO again.
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97

  Success &= SPISetup_MapSSOutput(SPI_SPI1, SPI1_SS_PIN);
  Success &= SPISetup_MapSDOutput(SPI_SPI1, SPI1_SDO_PIN);
  Success &= SPISetup_MapSDInput (SPI_SPI1, SPI1_SDI_PIN);

<<<<<<< HEAD
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
=======
  /* unlock PPS */
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 0;

  /* SDO1 -> RB5 */
  RPB5Rbits.RPB5R = 0b0011;

  /* SDI1 <- RB11  (THIS VALUE MAY NEED TO CHANGE ON SOME PIC32 VARIANTS) */
  SDI1Rbits.SDI1R = 0b0011;

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

  DB_printf("SPIService: SPI1 ON, BRG=%u\r\n", (unsigned)SPI_BRG_VALUE);
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
>>>>>>> a02604399845d76847dbf9cbbbdfbfcbbce53b97
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
