#ifndef REFLECTIVE_SENSE_SERVICE_H
#define REFLECTIVE_SENSE_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "ES_Configure.h"
#include "ES_Framework.h"

/*
  ReflectiveSenseService reads the Pololu reflective sensor using ADC.
  This is intentionally separate from MotorService so I can debug tape detection
  without touching motion code.
*/

bool InitReflectiveSenseService(uint8_t Priority);
bool PostReflectiveSenseService(ES_Event_t ThisEvent);
ES_Event_t RunReflectiveSenseService(ES_Event_t ThisEvent);

#endif
