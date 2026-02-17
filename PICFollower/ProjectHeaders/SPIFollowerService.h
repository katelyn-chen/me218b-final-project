#ifndef SPI_FOLLOWER_SERVICE_H
#define SPI_FOLLOWER_SERVICE_H

//#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */


/* Public Function Prototypes */
bool InitSPIFollowerService(uint8_t Priority);
bool PostSPIFollowerService(ES_Event_t ThisEvent);
ES_Event_t RunSPIFollowerService(ES_Event_t ThisEvent);



#endif /* SPI_FOLLOWER_SERVICE_H */
