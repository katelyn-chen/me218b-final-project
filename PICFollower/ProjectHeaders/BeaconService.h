#ifndef BEACON_SERVICE_H
#define BEACON_SERVICE_H

#include "ES_Configure.h"
#include "ES_Types.h"
#include "ES_Events.h"

/*
  BeaconService watches the IR beacon signal and decides “beacon found”.
  This service posts ES_BEACON_FOUND with EventParam encoding:
    - Beacon ID (G/B/R/L)
    - Bucket side (LEFT/RIGHT/UNKNOWN)
*/

/* beacon IDs from the Seesaw table */
typedef enum {
  BEACON_ID_NONE = 0,
  BEACON_ID_G    = 1,   /* 3333 Hz */
  BEACON_ID_B    = 2,   /* 1427 Hz */
  BEACON_ID_R    = 3,   /*  909 Hz */
  BEACON_ID_L    = 4    /* 2000 Hz */
} BeaconId_t;

/* bucket side classification */
typedef enum {
  BEACON_SIDE_UNKNOWN = 0,
  BEACON_SIDE_LEFT    = 1,
  BEACON_SIDE_RIGHT   = 2
} BeaconSide_t;

/* Pack into uint16_t: [15:8] BeaconId, [7:0] BeaconSide */
static inline uint16_t PackBeaconParam(BeaconId_t id, BeaconSide_t side)
{
  return (uint16_t)(((uint16_t)id << 8) | ((uint16_t)side & 0xFFu));
}

static inline void UnpackBeaconParam(uint16_t param, BeaconId_t *id, BeaconSide_t *side)
{
  if (id)   *id   = (BeaconId_t)((param >> 8) & 0xFFu);
  if (side) *side = (BeaconSide_t)(param & 0xFFu);
}

bool InitBeaconService(uint8_t Priority);
bool PostBeaconService(ES_Event_t ThisEvent);
ES_Event_t RunBeaconService(ES_Event_t ThisEvent);

#endif