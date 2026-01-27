/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef MotorService_H
#define MotorService_H

#include "ES_Types.h"
#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"  

// Public Function Prototypes

bool InitMotorService(uint8_t Priority);
bool PostMotorService(ES_Event_t ThisEvent);
ES_Event_t RunMotorService(ES_Event_t ThisEvent);

#endif /* MotorService_H */
