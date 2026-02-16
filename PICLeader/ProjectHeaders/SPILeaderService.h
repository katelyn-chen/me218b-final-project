#ifndef SPI_LEADER_SERVICE_H
#define SPI__LEADER_SERVICE_H

//#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */


/* Public Function Prototypes */
bool InitSPILeaderService(uint8_t Priority);
bool PostSPILeaderService(ES_Event_t ThisEvent);
ES_Event_t RunSPILeaderService(ES_Event_t ThisEvent);

#endif /* SPI_LEADER_SERVICE_H */
