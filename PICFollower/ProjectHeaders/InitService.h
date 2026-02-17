/****************************************************************************

  Header file for Init service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServInit_H
#define ServInit_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitInitService(uint8_t Priority);
bool PostInitService(ES_Event_t ThisEvent);
ES_Event_t RunInitService(ES_Event_t ThisEvent);

#endif /* ServInit_H */

