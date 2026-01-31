#ifndef BeaconService_H
#define BeaconService_H

#include "ES_Types.h"
#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"  

// Public Function Prototypes

bool InitBeaconService(uint8_t Priority);
bool PostBeaconService(ES_Event_t ThisEvent);
ES_Event_t RunBeaconService(ES_Event_t ThisEvent);

#endif /* BeaconService_H */