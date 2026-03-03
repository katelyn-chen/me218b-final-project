#ifndef SPI_LEADER_SERVICE_H
#define SPI_LEADER_SERVICE_H

//#include <stdint.h>
#include <stdbool.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */

/*----------------------------- DEFINING PINS FOR EVENT CHECKERS -----------------------------*/
#define InitButton  SPI_RPA2
#define ProgressLED SPI_RPA3

/*----------------------------- TypeDefs For Use Elsewhere -----------------------------*/
/*
  Field identification:
    FIELD_GREEN means beacons appear CCW in the BLRG cycle (with rotation offset).
    FIELD_BLUE  means beacons appear CCW in the GRLB cycle (with rotation offset).
*/
typedef enum {
  FIELD_UNKNOWN = 0,
  FIELD_GREEN   = 1,
  FIELD_BLUE    = 2
} Field_t;

typedef enum {
  FIRST_COLLECT,
  SECOND_COLLECT,
  OTHER_COLLECT
} CollectStatus_t;

/*----------------------------- Translate param helpers -----------------------------*/
typedef enum { TRANS_HALF = 0, TRANS_FULL = 1 } TransSpeed_t;
typedef enum { DIR_FWD = 0, DIR_REV = 1 } TransDir_t;

/*----------------------------- Rotate param helpers -----------------------------*/
typedef enum { ROT_45 = 0, ROT_90 = 1 } RotAngle_t;
typedef enum { ROT_CW = 0, ROT_CCW = 1 } RotDir_t;

/*----------------------------- Public Function Prototypes -----------------------------*/
bool InitSPILeaderService(uint8_t Priority);
bool PostSPILeaderService(ES_Event_t ThisEvent);
ES_Event_t RunSPILeaderService(ES_Event_t ThisEvent);

/*----------------------------- Public Function Definitions -----------------------------*/
static inline uint16_t PackTranslateParam(TransSpeed_t spd, TransDir_t dir)
{
  return (uint16_t)((spd & 0x1u) | ((dir & 0x1u) << 1));
}

static inline void UnpackTranslateParam(uint16_t p, TransSpeed_t *spd, TransDir_t *dir)
{
  *spd = (TransSpeed_t)(p & 0x1u);
  *dir = (TransDir_t)((p >> 1) & 0x1u);
}

static inline uint16_t PackRotateParam(RotAngle_t ang, RotDir_t dir)
{
  return (uint16_t)((ang & 0x1u) | ((dir & 0x1u) << 1));
}

static inline void UnpackRotateParam(uint16_t p, RotAngle_t *ang, RotDir_t *dir)
{
  *ang = (RotAngle_t)(p & 0x1u);
  *dir = (RotDir_t)((p >> 1) & 0x1u);
}

/*----------------------------- SPI Commands -----------------------------*/
/* leader -> follower motion commands */
#define CMD_STOP                  0x00
#define CMD_TRANS_FWD             0x01
#define CMD_TRANS_BWD             0x02
#define CMD_ROT_CW_90             0x03
#define CMD_ROT_CCW_90            0x04
#define CMD_ROT_CCW_180           0x05
#define CMD_ROT_CW_180            0x06
#define CMD_TAPE_T_DETECT         0x07
#define CMD_LINE_FOLLOW           0x08

/* leader -> follower beacon/nav commands */
#define CMD_GET_BEACON_FREQ       0x11
#define CMD_INIT_ORIENT           0x12

/* follower -> leader status bytes */
#define CMD_SIDE_FOUND_GREEN      0x13
#define CMD_SIDE_FOUND_BLUE       0x14
#define CMD_BEACON_R_FOUND        0x15
#define CMD_BEACON_L_FOUND        0x16
#define CMD_BEACON_G_FOUND        0x17
#define CMD_BEACON_B_FOUND        0x18

/* follower -> encoder commands */
#define CMD_ENCODER_ALIGN_COLLECT 0x19
#define CMD_ENCODER_ALIGN_DISPENSE 0x20
#define CMD_ENCODER_FIRST_ALIGN   0x21
#define CMD_MOVE_DONE             0x22
#define CMD_ALIGN_COLLECT         0x23
#define CMD_ENCODER_FIRST_FWD     0x24
#define CMD_COLLECT_BACK          0x25
#define CMD_COLLECT_FWD           0x26
#define CMD_FWD_AFTER_T           0x27

#define CMD_END_GAME              0x99
#define CMD_QUERY                 0xAA
#define CMD_NOOP                  0xFF
#define CMD_TESTING               0x10

/* collect/dispense sync (leader owns servos, follower freezes motors) */
#define CMD_FIRST_COLLECT_START   0x30
#define CMD_DISPENSE_START        0x31

/* optional unfreeze hooks */
#define CMD_FIRST_COLLECT_DONE    0x34
#define CMD_DISPENSE_DONE         0x35
#define CMD_SECOND_COLLECT_DONE   0x36
#define CMD_OTHER_COLLECT_DONE    0x37
#define CMD_SECOND_COLLECT_START  0x38
#define CMD_OTHER_COLLECT_START   0x39
#define CMD_FIRST_COLLECT_ARM_UP  0x40
#define CMD_FIRST_COLLECT_GRAB    0x41

#endif /* SPI_LEADER_SERVICE_H */
