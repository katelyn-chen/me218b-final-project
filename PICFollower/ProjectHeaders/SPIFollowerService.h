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

#endif /* SPI_FOLLOWER_SERVICE_H */
