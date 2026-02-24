#ifndef DISPENSE_SERVICE_H
#define DISPENSE_SERVICE_H

#include "ES_Configure.h"
#include "ES_Types.h"
#include "ES_Events.h"

/*
  DispenseService (Leader PIC)
    - Always runs 3-ball + 3-ball dispense pattern
    - Internal toggle:
        1st call -> gate half open
        2nd call -> gate full open
*/

bool InitDispenseService(uint8_t Priority);
bool PostDispenseService(ES_Event_t ThisEvent);
ES_Event_t RunDispenseService(ES_Event_t ThisEvent);

#endif