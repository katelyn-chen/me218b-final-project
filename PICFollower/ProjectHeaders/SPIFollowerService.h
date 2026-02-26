#ifndef SPI_FOLLOWER_SERVICE_H
#define SPI_FOLLOWER_SERVICE_H

//#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */


/*----------------------------- Public Function Prototypes ----------------------------*/
bool InitSPIFollowerService(uint8_t Priority);
bool PostSPIFollowerService(ES_Event_t ThisEvent);
ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent);

/*----------------------------- Pin Mapping Definitions ----------------------------*/
#define TapeSensor1 SPI_RPB4
#define TapeSensor2 SPI_RPB5
#define TapeSensor3  SPI_RPB11
#define TapeSensor4 SPI_RPB12
#define TapeSensor5 SPI_RPB13
#define UltrasonicEcho SPI_RPB10
#define UltrasonicTrigger SPI_RPA0  // check this one!

#define SDIPin SPI_RPB8
#define SDOPin SPI_RPA4
#define SCKPin SPI_RPB14
#define SSPin SPI_RPB15

/*----------------------------- Typedefs ----------------------------*/

typedef enum {
    TAPE_CENTERED,
    TAPE_OFF_CENTER_RIGHT,
    TAPE_OFF_CENTER_LEFT,
    FULL_T,
    RIGHT_CORNER,
    LEFT_CORNER,
    NO_TAPE
} TapeStatus_t;

/*----------------------------- SPI Commands -----------------------------*/

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

#define CMD_END_GAME              0x99
#define CMD_QUERY                 0xAA
#define CMD_NOOP                  0xFF
#define CMD_TESTING               0x10

/* collect/dispense sync (leader owns servos, follower freezes motors) */
#define CMD_COLLECT_START         0x30
#define CMD_DISPENSE_START        0x31

/* optional unfreeze hooks */
#define CMD_COLLECT_DONE          0x34
#define CMD_DISPENSE_DONE         0x35

#endif /* SPI_FOLLOWER_SERVICE_H */
