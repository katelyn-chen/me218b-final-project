#ifndef BEACON_SERVICE_H
#define BEACON_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "ES_Configure.h"
#include "ES_Framework.h"

/*
  BeaconService watches the IR beacon signal and decides “beacon found”.
  I’m keeping it event-only: this service posts ES_BEACON_FOUND and that’s it.
*/

bool InitBeaconService(uint8_t Priority);
bool PostBeaconService(ES_Event_t ThisEvent);
ES_Event_t RunBeaconService(ES_Event_t ThisEvent);

#endif
