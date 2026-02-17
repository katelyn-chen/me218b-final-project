/****************************************************************************

  Header file for Navigate service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServNavigate_H
#define ServNavigate_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitNavigateService(uint8_t Priority);
bool PostNavigateService(ES_Event_t ThisEvent);
ES_Event_t RunNavigateService(ES_Event_t ThisEvent);

#endif /* ServNavigate_H */

