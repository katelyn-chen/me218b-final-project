#ifndef COLLECT_SERVICE_H
#define COLLECT_SERVICE_H

#include "ES_Configure.h"
#include "ES_Types.h"
#include "ES_Events.h"

/*
  CollectService (Leader PIC)
    - Runs the dispenser ball pickup routine using servos
    - Uses SPILeaderService command bytes to request small robot nudges
    - Posts ES_COLLECT_DONE (optional) when finished
*/

bool InitCollectService(uint8_t Priority);
bool PostCollectService(ES_Event_t ThisEvent);
ES_Event_t RunCollectService(ES_Event_t ThisEvent);

#endif