#ifndef SPI_SERVICE_H
#define SPI_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "ES_Configure.h"
#include "ES_Framework.h"

/*
  SPIService talks to the Command Generator board.
  keep it as “bytes in -> ES events out” so MotorService stays clean.
*/

bool InitSPIService(uint8_t Priority);
bool PostSPIService(ES_Event_t ThisEvent);
ES_Event_t RunSPIService(ES_Event_t ThisEvent);

#endif
