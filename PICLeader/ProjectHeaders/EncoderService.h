/****************************************************************************

  Header file for Encoder service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServEncoder_H
#define ServEncoder_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitEncoderService(uint8_t Priority);
bool PostEncoderService(ES_Event_t ThisEvent);
ES_Event_t RunEncoderService(ES_Event_t ThisEvent);

// Pin Definitions
#define LeftEncoderA SPI_RPB11
//#define LeftEncoderB SPI_RPB9
#define RightEncoderA SPI_RPB2
//#define RightEncoderB SPI_RPB3

#endif /* ServEncoder_H */

