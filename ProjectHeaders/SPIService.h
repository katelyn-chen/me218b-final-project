#ifndef SPIService_H
#define SPIService_H

#include "ES_Types.h"
#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"  

// Public Function Prototypes

bool InitSPIService(uint8_t Priority);
bool PostSPIService(ES_Event_t ThisEvent);
ES_Event_t RunSPIService(ES_Event_t ThisEvent);

#endif /* SPIService_H */