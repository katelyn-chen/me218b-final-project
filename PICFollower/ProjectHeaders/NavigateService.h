/****************************************************************************

  Header file for Navigate service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServNavigate_H
#define ServNavigate_H

#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Events.h"
#include "ES_Types.h"     /* gets bool type for returns */

/* Translate param helpers */
typedef enum { TRANS_HALF = 0, TRANS_FULL = 1 } TransSpeed_t;
typedef enum { DIR_FWD = 0, DIR_REV = 1 } TransDir_t;

/* Rotate param helpers */
typedef enum { ROT_45 = 0, ROT_90 = 1 } RotAngle_t;
typedef enum { ROT_CW = 0, ROT_CCW = 1 } RotDir_t;

// Public Function Prototypes
static inline uint16_t PackTranslateParam(TransSpeed_t spd, TransDir_t dir)
{
  return (uint16_t)((spd & 0x1u) | ((dir & 0x1u) << 1));
}

static inline void UnpackTranslateParam(uint16_t p, TransSpeed_t *spd, TransDir_t *dir)
{
  *spd = (TransSpeed_t)(p & 0x1u);
  *dir = (TransDir_t)((p >> 1) & 0x1u);
}

static inline uint16_t PackRotateParam(RotAngle_t ang, RotDir_t dir)
{
  return (uint16_t)((ang & 0x1u) | ((dir & 0x1u) << 1));
}

static inline void UnpackRotateParam(uint16_t p, RotAngle_t *ang, RotDir_t *dir)
{
  *ang = (RotAngle_t)(p & 0x1u);
  *dir = (RotDir_t)((p >> 1) & 0x1u);
}

bool InitNavigateService(uint8_t Priority);
bool PostNavigateService(ES_Event_t ThisEvent);
ES_Event_t RunNavigateService(ES_Event_t ThisEvent);

#endif /* ServNavigate_H */

