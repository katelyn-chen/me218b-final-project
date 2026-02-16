#ifndef SPI_SERVICE_H
#define SPI_SERVICE_H

//#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */


/* Public Function Prototypes */
bool InitSPIService(uint8_t Priority);
bool PostSPIService(ES_Event_t ThisEvent);
ES_Event_t RunSPIService(ES_Event_t ThisEvent);

#endif /* SPI_SERVICE_H */
