/****************************************************************************
 Module
   NavigateService.c

 Revision
   1.0.0

 Description
   Navigate State Machine
****************************************************************************/

/*---------------------------- Module Types -------------------------------*/
typedef enum {
  WAITING_FOR_SIDE,
  FIRST_COLLECT,
  COLLECT_ALIGN,
  FIRST_DISPENSE,
  INIT_FIND_MIDDLE,
  ALIGN_MID_BUCKET,
  INIT_FIND_END,
  ALIGN_END_BUCKET,
  INIT_REPEAT_COLLECT,
  FIND_COLLECT
} NavigateState_t;

/*---------------------------- Module Variables --------------------------*/

static NavigateState_t curState;

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitNavigateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
****************************************************************************/
bool InitNavigateService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;

    if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
    PostNavigateService

 Parameters
     EF_Event_t ThisEvent, the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
****************************************************************************/
bool PostNavigateService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunNavigateService
****************************************************************************/

ES_Event_t RunNavigateService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (curState) {

    case WAITING_FOR_SIDE:

      if (ThisEvent.EventType == ES_SIDE_INDICATED) {

        if (ThisEvent.EventParam == LEFT) {
          // turn 45 CCW
        } else {
          // turn 45 CW
        }

        curState = FIRST_COLLECT;
      }
      break;


    case FIRST_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {

        ES_Event_t AlignEvent;
        AlignEvent.EventType = ES_ALIGN_COLLECT;
        AlignEvent.EventParam = FIRST;
        PostNavigateService(AlignEvent);

        curState = COLLECT_ALIGN;
      }
      break;


    case COLLECT_ALIGN:

      if (ThisEvent.EventType == ES_ALIGN_COLLECT) {
        StartCollectSM(ThisEvent.EventParam);
      }

      if (ThisEvent.EventType == ES_FIND_BUCKET) {

        if (ThisEvent.EventParam == FIRST) {
          curState = FIRST_DISPENSE;
        }
        else if (ThisEvent.EventParam == MIDDLE) {
          curState = INIT_FIND_MIDDLE;
        }
        else if (ThisEvent.EventParam == END) {
          curState = INIT_FIND_END;
        }
      }
      break;


    case FIRST_DISPENSE:

      if (ThisEvent.EventType == ES_DISPENSE) {
        StartDispenseSM(FULL);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_FIND_MIDDLE;
      }
      break;


    case INIT_FIND_MIDDLE:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = ALIGN_MID_BUCKET;
      }
      break;


    case ALIGN_MID_BUCKET:

      if (ThisEvent.EventType == ES_DISPENSE) {
        StartDispenseSM(SPLIT1);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_FIND_END;
      }
      break;


    case INIT_FIND_END:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = ALIGN_END_BUCKET;
      }
      break;


    case ALIGN_END_BUCKET:

      if (ThisEvent.EventType == ES_DISPENSE) {
        StartDispenseSM(SPLIT2);
      }

      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE) {
        curState = INIT_REPEAT_COLLECT;
      }
      break;


    case INIT_REPEAT_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {
        curState = FIND_COLLECT;
      }
      break;


    case FIND_COLLECT:

      if (ThisEvent.EventType == ES_T_DETECTED) {

        ES_Event_t AlignEvent;
        AlignEvent.EventType = ES_ALIGN_COLLECT;
        AlignEvent.EventParam = OTHER;
        PostNavigateService(AlignEvent);

        curState = COLLECT_ALIGN;
      }
      break;
  }

  return ReturnEvent;
}
