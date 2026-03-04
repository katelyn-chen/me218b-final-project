/****************************************************************************
 Module
   NavigateService.c

 Revision
   1.0.0

 Description
   Navigate State Machine
****************************************************************************/
#include "NavigateService.h"
#include "SPIFollowerService.h"
#include "BeaconService.h"
#include "PIC32_SPI_HAL.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "dbprintf.h"

/*---------------------------- Module Defines -------------------------------*/

#define PBCLK_HZ            20000000u
#define PWM_FREQ_HZ         7000u

/* Timer2 @ 10kHz PWM with prescale 1:1 -> PR2 = 1999 */
#define T2_PRESCALE_BITS    (0b000u) /* 1:1 */
#define T2_PRESCALE_VAL     1u
#define PR2_VALUE           ((PBCLK_HZ / (T2_PRESCALE_VAL * PWM_FREQ_HZ)) - 1u)

/* duty presets (percent 0..100) */
#define DUTY_STOP           0u
#define DUTY_TRANS_TAPE_DET 25u
#define DUTY_TRANS_HALF     30u
#define DUTY_TRANS_FULL     85u
#define DUTY_ROTATE         40u
#define DUTY_SEARCH         30u
#define TAPE_BASE_DUTY      DUTY_TRANS_TAPE_DET
#define TAPE_CORR_DUTY      8u   // steering correction amount
#define TAPE_LOST_DUTY      15u   // slow search when tape lost

/* Timer Values */
#define CHKPT3_FWD                5000u
#define ROTATE_45_TIME_MS         550u
#define ROTATE_90_TIME_MS         1100u
#define ROTATE_FIRST_COLLECT_MS   400u
#define VEER_AWAY_FROM_WALL       1000u
#define BACKUP_RAMPUP_MS          500
#define SEARCH_TIME               10000
#define TURN_DELAY_PRE_LF         2700 // turn before starting to line follow, 4500 
#define FIND_T_TIMER_BLOCK        5000
#define LINE_FOLLOWING_BLOCKING_TIMER   500 // DO NOT CHANGE
#define BACK_UP_PRE_COLLECT_ACTIVE 3500
#define BACK_COLLECT_TIME          900
#define FWD_ADJUST_COLLECT         300
#define LF_POSTING_DELAY           20




#define BUCKET_APPROACH_MS    4000   // SOY  tune this


/*---------------------------- Module Types -------------------------------*/
typedef enum {
  DEBUG,
  INIT_ORIENT,
  WAITING_FOR_SIDE,
  INIT_COAL_DISP_SEARCH,
  COLLECT_POST_T,
  START_LOOKING_FOR_T,
  COLLECT_ALIGN,
  COLLECT_ACTIVE,
  FIRST_DISPENSE,
  INIT_FIND_MIDDLE,
  MOVE_TO_MID_BUCKET,
  ALIGN_MID_BUCKET,
  INIT_FIND_END,
  ALIGN_END_BUCKET,
  INIT_REPEAT_COLLECT,
  FIND_COLLECT
} NavigateState_t;

typedef enum {
  ORIENT_IDLE,
  ORIENT_BEACON_SWEEP,
  ORIENT_DONE
} InitOrientState_t;

typedef enum {
  COLLECT_BACK,
  COLLECT_FWD,
  COLLECT_START
} InitCollectState_t;

typedef enum {
    FIRST,
    MIDDLE,
    END
} TapeTState_t;

/*
  Field identification:
    FIELD_GREEN means beacons appear CCW in the BLRG cycle (with rotation offset).
    FIELD_BLUE  means beacons appear CCW in the GRLB cycle (with rotation offset).
*/
typedef enum {
  FIELD_UNKNOWN = 0,
  FIELD_GREEN   = 1,
  FIELD_BLUE    = 2
} Field_t;

typedef enum {
    FOLLOW_FWD,
    FOLLOW_REV
} FollowDir_t;
/*---------------------------- Module Variables --------------------------*/
static uint8_t             MyPriority;
static NavigateState_t     curState;
static InitOrientState_t   orientState;
static InitCollectState_t  collectState;
static Field_t             field = FIELD_UNKNOWN;
static FollowDir_t followDir;
static uint16_t            duty = DUTY_STOP;

static uint8_t midMoveStage = 0;

//static int8_t LineFollowDirection = 1;   // +1 = forward, -1 = reverse
static bool coalSearchFlag  = 0;
static bool following = 1;
static bool collectStarted = 0;
static bool dispenseRequested = false;
static bool bucketApproachActive = false;
static bool correctingForLF = true;

static bool lookingForT = 0;
static bool beginfwd = 0;

static TapeStatus_t prevTapeState = NO_TAPE;

/* beacon sweep history (store unique transitions only) */
#define BEACON_SEQ_MAX 8u
static BeaconId_t Seq[BEACON_SEQ_MAX];
static uint8_t SeqLen = 0u;

/*---------------------------- Private Functions --------------------------*/
static void InitPinsAndPPS(void);
static void InitTimer2ForPWM(void);
static void InitOCsForPWM(void);

static void SendDispenseStart(void);

static void StopMotors(void);
static void SetMotor1(int16_t dutySignedPercent); /* left motor */
static void SetMotor2(int16_t dutySignedPercent); /* right motor */

static uint16_t DutyPercentToOCrs(uint16_t dutyPercent);

static void DoRotate(uint16_t rotateParam);
static void DoTranslate(uint16_t translateParam);
static void StartTapeDetect(void);
static void StartBeaconAlignSearch(void);

static void ResetBeaconSequence(void);
static bool PushBeaconIfNew(BeaconId_t id);
static Field_t DetermineFieldFromSequence(void);

/* Tape functions */
static void LineFollow(ES_Event_t ThisEvent, FollowDir_t followDirection);
static void SquareUpOnT(ES_Event_t ThisEvent);

/*------------------------------ Module Code ------------------------------*/

bool InitNavigateService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;

  InitPinsAndPPS();
  InitTimer2ForPWM();
  InitOCsForPWM();

  curState = INIT_ORIENT;
  orientState = ORIENT_IDLE;
  field = FIELD_UNKNOWN;
  duty = DUTY_STOP;

  ResetBeaconSequence();
  StopMotors();

  DB_printf("Navigate Service: init done\r\n");

  ThisEvent.EventType  = ES_INIT;
  ThisEvent.EventParam = 0;

  return ES_PostToService(MyPriority, ThisEvent);
}

bool PostNavigateService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunNavigateService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType  = ES_NO_EVENT;
  ReturnEvent.EventParam = 0;

  ES_Event_t cmdEvent;
  cmdEvent.EventType = ES_CMD_REQ;
  
  ES_Event_t startBeaconSearch;
  startBeaconSearch.EventType = ES_BEACON_SIGNAL;

  if (ThisEvent.EventType == ES_END_GAME)
  {
    DB_printf("Stopping Motors\r\n");
    StopMotors();
    return ReturnEvent;
  }

  switch (curState)
  {
    case INIT_ORIENT:
    {
      switch (orientState)
      {
        /* START SWITCHING ON ORIENT STATE */
        case ORIENT_IDLE:
        {
            /* Move from idle to turning for beacon upon start button press */
            if (ThisEvent.EventType == ES_INIT) {
            StopMotors();
            }
            
            if (ThisEvent.EventType == ES_ALIGN_ULTRASONICS)
            {
                DB_printf("Starting search for beacons to determine side\r\n");
                orientState = ORIENT_BEACON_SWEEP;
                field = FIELD_UNKNOWN;
                ResetBeaconSequence();
                StartBeaconAlignSearch();
                PostBeaconService(startBeaconSearch);
                ES_Timer_InitTimer(BEACON_TIMER, SEARCH_TIME);
            }
            break;
            /* END ORIENT IDLE */
        }

        case ORIENT_BEACON_SWEEP:
        {
            /* Continues turning to search for the beacon until beacon is found.
             * Reacts to the front left and front right limit switches being hit */
            
            
            /* LIMIT SWITCH REACTIONS!! */
            if (ThisEvent.EventType == ES_FRONT_LEFT_LIMIT_TRIGGER) {
                /* Hit front left limit switch */
                if (PORTAbits.RA3) {
                    DB_printf("The front left limit switch was hit but no wall detected in front! veering fwd and right\r\n");
                    SetMotor1(-(int16_t)DUTY_TRANS_HALF*0.5);
                    SetMotor2(-(int16_t)DUTY_TRANS_FULL*0.8);
                    ES_Timer_InitTimer(MOTOR_TIMER, VEER_AWAY_FROM_WALL);
                
                } else {
                    DB_printf("The front left limit switch was hit and a wall was detected in front! veering back and right\r\n");
                    SetMotor1((int16_t)DUTY_TRANS_HALF*0.5);
                    SetMotor2((int16_t)DUTY_TRANS_FULL*0.8);
                    ES_Timer_InitTimer(MOTOR_TIMER, VEER_AWAY_FROM_WALL);
                }
            }
            
            if (ThisEvent.EventType == ES_BACK_RIGHT_LIMIT_TRIGGER) {
                /* Hit back right limit switch*/
                if (PORTAbits.RA3) {
                    DB_printf("The back right limit switch was hit but no wall detected in front! veering fwd and left\r\n");
                    SetMotor1(-(int16_t)DUTY_TRANS_HALF*0.5);
                    SetMotor2(-(int16_t)DUTY_TRANS_FULL*0.8);
                    ES_Timer_InitTimer(MOTOR_TIMER, VEER_AWAY_FROM_WALL);
                
                } else {
                    DB_printf("The back right limit switch was hit and a wall was detected in front! veering back and right\r\n");
                    SetMotor1((int16_t)DUTY_TRANS_HALF*0.5);
                    SetMotor2((int16_t)DUTY_TRANS_FULL*0.8);
                    ES_Timer_InitTimer(MOTOR_TIMER, VEER_AWAY_FROM_WALL);
                }
            }
            /* END LIMIT SWITCH REACTIONS!! */
            
            /* Motor timer to resume beacon search after reacting to limit switches
             * by veering!! */
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER) {
                StartBeaconAlignSearch();
                PostBeaconService(startBeaconSearch);
            }
            
            /* Beacon timer that tells bot to drive forward if we ever get stuck in a
             * beacon search loop so do we don't spin endlessly!! */
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BEACON_TIMER) {
                DoTranslate(PackTranslateParam(TRANS_HALF, DIR_FWD));
                ES_Timer_InitTimer(MOTOR_TIMER, 1000);
                
            }
            
            /* DEBUGGING KEY PRESSES */
            if (ThisEvent.EventType == ES_NEW_KEY && ThisEvent.EventParam == '1') {
                orientState = ORIENT_DONE;
                DB_printf("Moving to orient done state because key pressed\r\n");
                DB_printf("L detected from button press! We are on GREEN field\r\n");
                field = FIELD_BLUE;
                cmdEvent.EventParam = CMD_SIDE_FOUND_BLUE;
                PostSPIFollowerService(cmdEvent); 
            }
            
            if (ThisEvent.EventType == ES_NEW_KEY && ThisEvent.EventParam == '2') {
                DoTranslate(PackTranslateParam(DUTY_TRANS_HALF, DIR_FWD));
                cmdEvent.EventParam = CMD_ENCODER_FIRST_FWD;
                PostSPIFollowerService(cmdEvent);
                DB_printf("testing translate \r\n");
            }
            
            if (ThisEvent.EventType == ES_NEW_KEY && ThisEvent.EventParam == '3') {
                DB_printf("Jumping into collect sequence! \r\n");
                cmdEvent.EventParam = CMD_ALIGN_COLLECT;
                PostSPIFollowerService(cmdEvent);
                DoTranslate(PackTranslateParam(TRANS_HALF, DIR_FWD));
                curState = COLLECT_ALIGN;
                collectState = COLLECT_START;
            }
            
            if (ThisEvent.EventType == ES_NEW_KEY && ThisEvent.EventParam == '4') {
                DB_printf("Testing line following, jumping to first collect\r\n");
                curState = FIRST_DISPENSE;
            }

            /* END DEBUGGING KEY PRESSES */
            
            /* Moves toward beacon and tells leader to move the flag to
             * indicate the beacon has been found! */
            if (ThisEvent.EventType == ES_BEACON_FOUND)
            {
              BeaconId_t id;
              BeaconSide_t side;
              UnpackBeaconParam((uint16_t)ThisEvent.EventParam, &id, &side);
              (void)side;
              DB_printf("Beacon found event posted from nav service, id: %d\r\n", id);
                if (id != BEACON_ID_NONE)
                {
                    /* Identifies side based on if it sees L/R, ignores other frequencies */
                    if (id == BEACON_ID_L) {
                        DB_printf("L detected! We are on GREEN field\r\n");
                        field = FIELD_GREEN;
                        cmdEvent.EventParam = CMD_SIDE_FOUND_GREEN;
                    } else if (id == BEACON_ID_R) {
                        DB_printf("R detected! We are on BLUE field\r\n");
                        field = FIELD_BLUE;
                        cmdEvent.EventParam = CMD_SIDE_FOUND_BLUE;
                    } else {
                        DB_printf("Other beacon detected, id = %d. Will keep rotating.\r\n", id);
                    }

                    if (field != FIELD_UNKNOWN)
                    {
                        DB_printf("Field determined: %u\r\n", (unsigned)field);
                        DB_printf("Moving towards L/R beacon\r\n");
                        // move towards the beacon
                        DoTranslate(PackTranslateParam(DUTY_TRANS_HALF, DIR_FWD));
                        PostSPIFollowerService(cmdEvent);
                    }
                }
            }
            /* END BEACON RESPONSE */

            /* Once the encoder says the bot is done moving towards the beacon,
             * the bot begins to turn to move towards a line */
            if (ThisEvent.EventType == ES_MOVE_DONE)
            {
              orientState = ORIENT_DONE;
              DB_printf("ORIENT_DONE: Rotating to face straight along line\r\n");
              DoRotate(PackRotateParam(ROT_90, ROT_CCW));
              cmdEvent.EventParam = CMD_ENCODER_FIRST_ALIGN;
              PostSPIFollowerService(cmdEvent);
            }
            
            break;
        }
        /* END ORIENT BEACON SWEEP */

        case ORIENT_DONE:
        {
          /* Once facing straight, move forward until two T's are detected */
            if (ThisEvent.EventType ==  ES_MOVE_DONE) {
                /* Done rotating to tape after beacon */ 
                curState = INIT_COAL_DISP_SEARCH;
                
                /* Manually setting motors here because we want to modify the duty cycles
                 a bit for driving straight */
                DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_FWD));
                ES_Timer_InitTimer(BEACON_TIMER, LINE_FOLLOWING_BLOCKING_TIMER);
                ES_Timer_InitTimer(MOTOR_TIMER, FIND_T_TIMER_BLOCK);
                following = 0;
            }
                break;
        }
        /* END ORIENT DONE */
        
        default:
          break;
          
        }
        break;
    }
    /* END SWITCHING CASES ON ORIENT STATE!! END INIT ORIENT */

    case INIT_COAL_DISP_SEARCH:
    {
        /* drive forward until T detected */
        
        if (ThisEvent.EventType == ES_TIMEOUT) {
            switch (ThisEvent.EventParam) {
                case BEACON_TIMER:
                {
                    DB_printf("Begin tape detect\r\n");
                    following = 1;
                    lookingForT = 0;
                    ES_Timer_StopTimer(BEACON_TIMER);
                    break;
                }
                
                case MOTOR_TIMER:
                {
                    DB_printf("Begin looking for T\r\n");
                    lookingForT = 1;
                    ES_Timer_StopTimer(MOTOR_TIMER);
                    break;
                }
                
                case LF_POSTING_TIMER:
                {
                    correctingForLF = 1;
                    break;
                }
            }
        }
      
      if (ThisEvent.EventType == ES_T_DETECTED && ThisEvent.EventParam == FULL_T && lookingForT )
      {
        // rotate clockwise to face away from dispenser, start timer before line following
//        DoRotate(PackRotateParam(ROT_90, ROT_CW));
        DB_printf("T found!! veering initially to start turning clockwise\r\n");
        following = 0;
        cmdEvent.EventParam = CMD_ROT_CW_180;
        ES_Timer_InitTimer(MOTOR_TIMER, VEER_AWAY_FROM_WALL*1.3);
        PostSPIFollowerService(cmdEvent);
        SetMotor1(-(int16_t)DUTY_TRANS_HALF*0.8);
        SetMotor2(-(int16_t)DUTY_TRANS_HALF*1.2);
        curState = START_LOOKING_FOR_T;
        lookingForT = 0;
      } else if (ThisEvent.EventType == ES_TAPE_DETECT && following)
      {
          if (correctingForLF) {
            correctingForLF = 0; 
            ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
            LineFollow(ThisEvent, FOLLOW_FWD);
          }
              
      }
      
      break;
    }
    /* END INIT COAL DISPNESE SEARCH - NOW FACING AWAY FROM DISPENSER */
    
//    case COLLECT_POST_T: {
//        if (ThisEvent.EventType == ES_TAPE_DETECT && following)
//        {
//            LineFollow(ThisEvent, FOLLOW_FWD);
//            prevTapeState = ThisEvent.EventParam;
//        }
//        
//        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER) {
//            curState = START_LOOKING_FOR_T;
//        }
//        
//        break;
//    }
    
    case START_LOOKING_FOR_T: {
        if (ThisEvent.EventType == ES_TIMEOUT) {
            switch (ThisEvent.EventParam) {
                case BEACON_TIMER:
                {
                    ES_Timer_InitTimer(BEACON_TIMER, BACK_UP_PRE_COLLECT_ACTIVE);
                    DB_printf("Starting back up\r\n");
                    DoTranslate(PackTranslateParam(TRANS_HALF*0.8, DIR_REV));
                    followDir = FOLLOW_REV;
                    collectState = COLLECT_START;
                    following = 1;
                    collectStarted = 0;            
                    curState = COLLECT_ALIGN;
                    break;
                }
                
                case MOTOR_TIMER:
                {
                    /* keep rotating, but now change state so we can start line following
                    * now that the timer had expired */
                     DB_printf("Done veering - turning on centroid now\r\n");
                     ES_Timer_StopTimer(MOTOR_TIMER);
                     SetMotor1((int16_t)DUTY_TRANS_HALF);
                     SetMotor2(-(int16_t)DUTY_TRANS_HALF*1.2);
                     following = 0;
                     ES_Timer_InitTimer(BEACON_TIMER, TURN_DELAY_PRE_LF); // 270 turn
                    break;
                }
                
                case LF_POSTING_TIMER:
                {
                    DB_printf("Toggling LF_POSTING_TIMER in START_LOOKING_FOR_T\r\n");
                    correctingForLF = 1;
                    ES_Timer_StopTimer(LF_POSTING_TIMER);
                    break;
                }
            }
        }
        
        if (ThisEvent.EventType == ES_T_DETECTED){};
        break;
    }
    /* END COLLECT POST T - NOW FACING DISPENSER */

    case COLLECT_ALIGN:
    {
        /* Bot should be moving backwards when it enters this state! */
        
        /* TIMER RESPONSES */
        if (ThisEvent.EventType == ES_TIMEOUT) {
            if (ThisEvent.EventParam == BEACON_TIMER) {
                /* can start moving forward! */
                DB_printf("Timer expired for backup - now moving forward. Posting start collect\r\n");
                collectStarted = 1; // mark collect as started
                SetMotor1(-(int16_t)DUTY_TRANS_HALF*1.5);
                SetMotor2(-(int16_t)DUTY_TRANS_HALF*0.5);
                followDir = FOLLOW_FWD;
                cmdEvent.EventParam = CMD_FIRST_COLLECT_START;
                PostSPIFollowerService(cmdEvent);
                lookingForT = 1;
                curState = COLLECT_ACTIVE;
            } else if (ThisEvent.EventParam == LF_POSTING_TIMER) {
                correctingForLF = 1;
            }
        }
        /* END TIMER RESPONSES */
        
//        if (ThisEvent.EventType == ES_T_DETECTED && !collectStarted) {
//            if (ThisEvent.EventParam == LEFT_CORNER) {
//                /* Once the T has been detected, bot moves back and lowers arm
//                 I added this flag because I only want this code to run once */
//                /* needs to move back bc too close*/
//                following = 1;
//                DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));
//                followDir = FOLLOW_REV;
//                cmdEvent.EventParam = CMD_ALIGN_COLLECT;
//                PostSPIFollowerService(cmdEvent);
//            }
//            
//            if (ThisEvent.EventParam == FULL_T) {
//                /* can start moving forward!*/
//                collectStarted = 1; // mark collect as started
//                following = 1;
//                followDir = FOLLOW_FWD;
//                cmdEvent.EventParam = CMD_FIRST_COLLECT_START;
//                PostSPIFollowerService(cmdEvent);
//                lookingForT = 1;
//                curState = COLLECT_ACTIVE;
//            }
        /* Line follow for  initially moving back from the dispenser */
        if (ThisEvent.EventType == ES_TAPE_DETECT && following)
        {
          if (correctingForLF) {
            correctingForLF = 0; 
            ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
            LineFollow(ThisEvent, FOLLOW_REV);
          }
        }
        
        break;
    }
    
    case COLLECT_ACTIVE:
    {
        /* Line follow for both nudge fwd and nudge back, as well as initially
         moving back from the dispenser. Arm should be down and bot should 
         * get command to nudge forward from the collect service soon after we enter
         * this state! (timer based in collect service) */
        
        /* Checking if we should stop moving back/forth */
        if (ThisEvent.EventType == ES_T_DETECTED && collectState == COLLECT_FWD && beginfwd)
        {
          /* we are looking for the left corner MAYBE change to FULL_T */
          /* need to start grabbing here! */
            beginfwd = 0;
            DB_printf("adjusting collect fwd alignment a bit!\r\n");
            ES_Timer_InitTimer(BEACON_TIMER, FWD_ADJUST_COLLECT);
            SetMotor1((int16_t)-DUTY_TRANS_HALF);
            SetMotor2((int16_t)-1.5*DUTY_TRANS_HALF);
            break;
        }
        
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER
                && collectState == COLLECT_BACK) {
            DB_printf("telling collect to move arm up\r\n");
            StopMotors();
            following = 0;
            cmdEvent.EventParam = CMD_FIRST_COLLECT_ARM_UP;
            PostSPIFollowerService(cmdEvent);
            break;
        }
        
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BEACON_TIMER
                && collectState == COLLECT_FWD) {
            DB_printf("telling collect to grab from timer!\r\n");
            ES_Timer_StopTimer(BEACON_TIMER);
            StopMotors();
            following = 0;
            cmdEvent.EventParam = CMD_FIRST_COLLECT_GRAB;
            PostSPIFollowerService(cmdEvent);
            break;
        }
        
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == LF_POSTING_TIMER) {
            correctingForLF = 0;
        }
        
        /* END checks to see if we should stop moving back/forth */
        
        /* EVENTS ORIGINATING FROM COLLECT SERVICE --> SPILeader --> SPIFollower */
        
        if (ThisEvent.EventType == ES_COLLECT_FWD) {
           /* Moving fwd to the ball dispenser needs to stop
            when it sees the t*/
            DB_printf("collect requesting forward!\r\n");
            cmdEvent.EventParam = CMD_COLLECT_FWD;
            PostSPIFollowerService(cmdEvent);
            DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_FWD));
            following = 1;
            collectState = COLLECT_FWD;
            followDir = FOLLOW_FWD;
            beginfwd = 1;
        }
        
        if (ThisEvent.EventType == ES_COLLECT_BACK) {
            /* Moving back from the ball dispenser */
            DB_printf("collect requesting backup!\r\n");
            cmdEvent.EventParam = CMD_COLLECT_BACK;
            PostSPIFollowerService(cmdEvent);
            ES_Timer_InitTimer(MOTOR_TIMER, BACK_COLLECT_TIME);
            DoTranslate(PackTranslateParam(TRANS_FULL, DIR_REV));
            
            /* Initiating backup ramp up - half the backup speed */
//            ES_Timer_InitTimer(MOTOR_TIMER, BACKUP_RAMPUP_MS);
//            SetMotor1((int16_t)DUTY_TRANS_FULL*1.15);
//            SetMotor2((int16_t)DUTY_TRANS_FULL*0.7);
            following = 1;
            collectState = COLLECT_BACK;
            followDir = FOLLOW_REV;
        }
        
//        /* This is for the backup ramp up - moving to full backup speed */
//        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER) {
//            SetMotor1((int16_t)DUTY_TRANS_FULL*1.2);
//            SetMotor2((int16_t)DUTY_TRANS_FULL*0.5);
//            ES_Timer_StopTimer(MOTOR_TIMER);
//        }
        
        /* END EVENTS ORIGINATING FROM COLLECT SERVICE */
        
        if (ThisEvent.EventType == ES_TAPE_DETECT && following)
        {
          if (correctingForLF) {
            correctingForLF = 0; 
            ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
            LineFollow(ThisEvent, followDir);
          }
        }
              
           
      if (ThisEvent.EventType == ES_FIND_BUCKET)
        {
          DB_printf("exiting collect\r\n");
          if (ThisEvent.EventParam == FIRST_COLLECT) {
            bucketApproachActive = false;
            dispenseRequested = false;
            curState = FIRST_DISPENSE;
            DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));
          }
          else if (ThisEvent.EventParam == SECOND_COLLECT) {
            curState = INIT_FIND_MIDDLE;
          }
          else if (ThisEvent.EventParam == OTHER_COLLECT) {
            curState = INIT_FIND_END;
          }
          break;
        }

        break; // MUST ADD HERE TO NOT SKIP TO dispense!!!!
        }

    case FIRST_DISPENSE:
        {
            
        if (!bucketApproachActive)
        {
            /* We want to start line following backwards here*/
//          ES_Timer_StopTimer(BUCKET_TIMER);
          DB_printf("FIRST bucket: starting timed approach\r\n");
          bucketApproachActive = true;
          dispenseRequested = false;

          following = 1;
          followDir = FOLLOW_REV;
          DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));

          ES_Timer_InitTimer(BUCKET_TIMER, BUCKET_APPROACH_MS);
        }
        
        /* Reverse line follow toward first bucket */
        /* After dispensing, transition to middle bucket search */

        if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
        {
          DB_printf("Dispense complete (first). Going to middle bucket.\r\n");

          dispenseRequested = false;
          bucketApproachActive = false;   // reset for next approach

          /* begin moving toward middle bucket (your existing behavior) */
          followDir = FOLLOW_REV;
          following = 1;
          DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));

          curState = INIT_FIND_MIDDLE;   /* IMPORTANT: go find the next T before turning */
        }

        /* Start the 1.5s approach window ONCE when you enter this behavior */

        /* When the timer expires, stop and dispense */
        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BUCKET_TIMER)
        {
          DB_printf("bucket timeout");
          StopMotors();
          following = 0;
          ES_Timer_StopTimer(BUCKET_TIMER);

          if (!dispenseRequested)
          {
            dispenseRequested = true;
            DB_printf("FIRST bucket: timed stop -> request dispense\r\n");
            SendDispenseStart();
          }
        }
        
        if (ThisEvent.EventType == ES_TAPE_DETECT && following)
        {
          DB_printf("telling collect to grab from tape!\r\n");
          if (correctingForLF) {
            correctingForLF = 0; 
            ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
            LineFollow(ThisEvent, FOLLOW_REV);
          }
        }
        break;
      }

    
   

  

    case INIT_FIND_MIDDLE:
    {
      if (ThisEvent.EventType == ES_T_DETECTED && ThisEvent.EventParam == FULL_T)
      {
        DB_printf("MIDDLE: found T, starting 2-turn maneuver\r\n");

        /* stage 0: do first 90-degree turn */
        midMoveStage = 0;
        following = 0;
        ES_Timer_StopTimer(MOTOR_TIMER);

        /* turn 90 degrees right (your existing motor pattern) */
        SetMotor1((int16_t)DUTY_TRANS_HALF);
        SetMotor2(-(int16_t)(DUTY_TRANS_HALF * 1.2));
        ES_Timer_InitTimer(MOTOR_TIMER, ROTATE_90_TIME_MS);

        curState = MOVE_TO_MID_BUCKET;
      }
      break;
    }

    case MOVE_TO_MID_BUCKET:
    {
      /* stage 0 -> waiting for first 90 turn to finish */
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER && midMoveStage == 0)
      {
        DB_printf("MIDDLE: first 90 done, now line follow to next T\r\n");

        ES_Timer_StopTimer(MOTOR_TIMER);

        /* start reverse line following again until we hit the next FULL_T */
        following = 1;
        followDir = FOLLOW_REV;
        DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));

        midMoveStage = 1;
        break;
      }

      /* stage 1 -> line follow until the next FULL_T, then do second 90 turn */
      if (ThisEvent.EventType == ES_TAPE_DETECT && following && midMoveStage == 1)
      {
        /* keep line following while looking for the next T */
        if (correctingForLF) {
          correctingForLF = 0;
          ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
          LineFollow(ThisEvent, FOLLOW_REV);
        }
      }

      if (ThisEvent.EventType == ES_T_DETECTED && ThisEvent.EventParam == FULL_T && midMoveStage == 1)
      {
        DB_printf("MIDDLE: second T found, doing second 90\r\n");

        following = 0;
        ES_Timer_StopTimer(MOTOR_TIMER);

        /* turn 90 left (your intended motor pattern, fixed syntax) */
        SetMotor1(-(int16_t)DUTY_TRANS_HALF);
        SetMotor2((int16_t)(DUTY_TRANS_HALF * 1.2));
        ES_Timer_InitTimer(MOTOR_TIMER, ROTATE_90_TIME_MS);

        midMoveStage = 2;
        break;
      }

      /* stage 2 -> second 90 done, now we are ready to approach middle bucket + dispense */
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER && midMoveStage == 2)
      {
        DB_printf("MIDDLE: second 90 done, entering ALIGN_MID_BUCKET\r\n");

        ES_Timer_StopTimer(MOTOR_TIMER);

        /* reset approach flags so ALIGN_MID_BUCKET starts its timed approach cleanly */
        bucketApproachActive = false;
        dispenseRequested = false;

        /* start moving backwards again (ALIGN_MID_BUCKET expects reverse approach) */
        following = 1;
        followDir = FOLLOW_REV;
        DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));

        curState = ALIGN_MID_BUCKET;
        break;
      }

      break;
    }

    case ALIGN_MID_BUCKET:
    {
      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
      {
        DB_printf("Dispense complete (middle).\r\n");

        dispenseRequested = false;
        bucketApproachActive = false;

        curState = INIT_FIND_END;  // or next phase
        break;
      }

      /* Start timed approach ONCE */
      if (!bucketApproachActive)
      {
        ES_Timer_StopTimer(BUCKET_TIMER);
        DB_printf("MIDDLE bucket: starting timed approach\r\n");
        bucketApproachActive = true;
        dispenseRequested = false;

        following = 1;
        followDir = FOLLOW_REV;
        DoTranslate(PackTranslateParam(TRANS_TAPE, DIR_REV));

        ES_Timer_InitTimer(BUCKET_TIMER, BUCKET_APPROACH_MS);
      }

      /* Timer expiry -> stop and dispense */
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BUCKET_TIMER)
      {
        StopMotors();
        following = 0;
        ES_Timer_StopTimer(BUCKET_TIMER);

        if (!dispenseRequested)
        {
          dispenseRequested = true;
          DB_printf("MIDDLE bucket: timed stop -> request dispense\r\n");
          SendDispenseStart();   // same command, leader toggles 90/180 internally
        }
      }

      /* Reverse line follow toward middle bucket */
      if (ThisEvent.EventType == ES_TAPE_DETECT && following)
      {
        if (correctingForLF) {
          correctingForLF = 0;
          ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
          LineFollow(ThisEvent, FOLLOW_REV);
        }
      }

      break;
    }

    case INIT_FIND_END:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = ALIGN_END_BUCKET;
      }
      break;
    }

    case ALIGN_END_BUCKET:
    {
      if (ThisEvent.EventType == ES_DISPENSE_COMPLETE)
      {
        curState = INIT_REPEAT_COLLECT;
      }
      break;
    }

    case INIT_REPEAT_COLLECT:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = FIND_COLLECT;
      }
      break;
    }

    case FIND_COLLECT:
    {
      if (ThisEvent.EventType == ES_T_DETECTED)
      {
        curState = COLLECT_ALIGN;
      }
      break;
    }

    case WAITING_FOR_SIDE:
    default:
      break;
  }

  return ReturnEvent;
}

/*=========================== INIT HELPERS ============================*/
static void InitPinsAndPPS(void)
{
  /* motor pins as digital outputs (update these mappings if motor wiring changes) */

  PIN_MapPinOutput(LEFT_IN1);
  PIN_MapPinOutput(LEFT_IN2);
  PIN_MapPinOutput(RIGHT_IN3);
  PIN_MapPinOutput(RIGHT_IN4);

  // Selecting OC Channels
  #define PPS_FN_PWM 0b0101
  RPA1Rbits.RPA1R = PPS_FN_PWM;
  RPB9Rbits.RPB9R = PPS_FN_PWM;
  RPB3Rbits.RPB3R = PPS_FN_PWM;
  RPB2Rbits.RPB2R = PPS_FN_PWM;
}

static void InitTimer2ForPWM(void)
{
  T2CON = 0;
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = T2_PRESCALE_BITS;
  TMR2 = 0;
  PR2 = (uint16_t)PR2_VALUE;
  T2CONbits.ON = 1;
}

static void InitOCsForPWM(void)
{
  /* all OCs use Timer2 (OCTSEL=0) and PWM mode (OCM=0b110) */
  OC1CON = 0; OC1R = 0; OC1RS = 0; OC1CONbits.OCTSEL = 0; OC1CONbits.OCM = 0b110; OC1CONbits.ON = 1;
  OC2CON = 0; OC2R = 0; OC2RS = 0; OC2CONbits.OCTSEL = 0; OC2CONbits.OCM = 0b110; OC2CONbits.ON = 1;
  OC3CON = 0; OC3R = 0; OC3RS = 0; OC3CONbits.OCTSEL = 0; OC3CONbits.OCM = 0b110; OC3CONbits.ON = 1;
  OC4CON = 0; OC4R = 0; OC4RS = 0; OC4CONbits.OCTSEL = 0; OC4CONbits.OCM = 0b110; OC4CONbits.ON = 1;
}

static void SendDispenseStart(void)
{
  ES_Event_t e;
  e.EventType  = ES_CMD_REQ;
  e.EventParam = CMD_DISPENSE_START;
  PostSPIFollowerService(e);
}

/*=========================== MOTOR OUTPUT ============================*/
static uint16_t DutyPercentToOCrs(uint16_t dutyPercent)
{
  if (dutyPercent > 100u) dutyPercent = 100u;
  return (uint16_t)(((uint32_t)dutyPercent * (uint32_t)(PR2)) / 100u);
}

static void StopMotors(void)
{
  DB_printf("Stopping motors\r\n");
  OC2RS = 0;
  OC3RS = 0;
  OC1RS = 0;
  OC4RS = 0;
}

static void SetMotor1(int16_t dutySignedPercent)
//  right motor
{
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);

  if (dutySignedPercent > 0)
  {
    OC2RS = ocrs;
    OC3RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC2RS = 0;
    OC3RS = ocrs;
  }
  else
  {
    OC2RS = 0;
    OC3RS = 0;
  }
}

static void SetMotor2(int16_t dutySignedPercent)
{
  uint16_t mag = (dutySignedPercent < 0) ? (uint16_t)(-dutySignedPercent) : (uint16_t)dutySignedPercent;
  uint16_t ocrs = DutyPercentToOCrs(mag);
//  DB_printf("Duty*PR2: %d\r\n", ocrs);
//  DB_printf("Motor2 Duty: %d\r\n", dutySignedPercent);

  if (dutySignedPercent > 0)
  {
    OC1RS = ocrs;
    OC4RS = 0;
  }
  else if (dutySignedPercent < 0)
  {
    OC1RS = 0;
    OC4RS = ocrs;
  }
  else
  {
    OC1RS = 0;
    OC4RS = 0;
  }
}

/*=========================== MOTION PRIMITIVES ============================*/
static void DoTranslate(uint16_t translateParam)
{
  TransSpeed_t spd;
  TransDir_t dir;
  UnpackTranslateParam(translateParam, &spd, &dir);
  

  switch (spd)
  {
    case TRANS_FULL:
      duty = DUTY_TRANS_FULL;
      break;

    case TRANS_HALF:
      duty = DUTY_TRANS_HALF;
      break;

    case TRANS_TAPE:
      duty = DUTY_TRANS_TAPE_DET;
      break;

    default:
      duty = DUTY_STOP;
      break;
  }

  {
    int16_t signedDuty = (dir == DIR_FWD) ? -(int16_t)duty : (int16_t)duty;
    SetMotor1(signedDuty);
    SetMotor2(signedDuty);
  }
}

static void DoRotate(uint16_t rotateParam)
{
  RotAngle_t ang;
  RotDir_t dir;
  UnpackRotateParam(rotateParam, &ang, &dir);
  (void)ang;

  if (dir == ROT_CW)
  {
    SetMotor2(-(int16_t)DUTY_ROTATE);
    SetMotor1((int16_t)DUTY_ROTATE);
  }
  else
  {
    SetMotor2((int16_t)DUTY_ROTATE);
    SetMotor1(-(int16_t)DUTY_ROTATE);
  }
}

static void StartTapeDetect(void)
{
  SetMotor1((int16_t)DUTY_TRANS_HALF);
  SetMotor2((int16_t)DUTY_TRANS_HALF);
}

static void LineFollow(ES_Event_t ThisEvent, FollowDir_t followDirection)
{
    int16_t leftDuty  = 0;
    int16_t rightDuty = 0;

    switch (ThisEvent.EventType)
    {
        case ES_TAPE_DETECT:
        {
            switch (ThisEvent.EventParam)
            {
                case TAPE_CENTERED:
                    leftDuty  =  TAPE_BASE_DUTY;
                    rightDuty =  TAPE_BASE_DUTY;
                    break;

                case TAPE_OFF_CENTER_LEFT:
                    // drifting LEFT ? steer RIGHT
                    leftDuty  =  TAPE_BASE_DUTY - TAPE_CORR_DUTY;
                    rightDuty =  TAPE_BASE_DUTY + TAPE_CORR_DUTY;
                    break;

                case TAPE_OFF_CENTER_RIGHT:
                    // drifting RIGHT ? steer LEFT
                    leftDuty  =  TAPE_BASE_DUTY + TAPE_CORR_DUTY;
                    rightDuty =  TAPE_BASE_DUTY - TAPE_CORR_DUTY;
                    break;

                case NO_TAPE:
                    leftDuty  =  TAPE_LOST_DUTY;
                    rightDuty = -TAPE_LOST_DUTY;
                    break;
                    
                case TAPE_EXTREME_OFF_CENTER_LEFT:
                    leftDuty  =  TAPE_BASE_DUTY - 1.5*TAPE_CORR_DUTY;
                    rightDuty =  TAPE_BASE_DUTY + 1.5*TAPE_CORR_DUTY;
                    break;
                    
                case TAPE_EXTREME_OFF_CENTER_RIGHT:
                    leftDuty  =  TAPE_BASE_DUTY + 1.5*TAPE_CORR_DUTY;
                    rightDuty =  TAPE_BASE_DUTY - 1.5*TAPE_CORR_DUTY;
                    break;

                default:
                    break;
            }

            /*
            * If reversing, swap steering correction
            */
           if (followDirection == FOLLOW_REV)
           {
               int16_t temp = leftDuty;
               leftDuty  = rightDuty;
               rightDuty = temp;
           }

           /*
            * Motor convention:
            * NEGATIVE = forward
            * POSITIVE = backward
            */
           int8_t motorSign;

           if (followDirection == FOLLOW_FWD) {
               motorSign = -1;
           } else {
               motorSign = 1;
           }
           
//           if (correctingForLF) {
//            correctingForLF = 0;
            SetMotor1(leftDuty  * motorSign);
            SetMotor2(rightDuty * motorSign);
//            ES_Timer_InitTimer(LF_POSTING_TIMER, LF_POSTING_DELAY);
//           }

            break;
        }

        default:
            break;
    }
}

static void StartBeaconAlignSearch(void)
{
  /* CCW in-place sweep to collect beacon ordering */
  SetMotor1(-(int16_t)DUTY_SEARCH);
  SetMotor2((int16_t)DUTY_SEARCH);
}

/*=========================== BEACON ORDER LOGIC ============================*/
static void ResetBeaconSequence(void)
{
  for (uint8_t i = 0u; i < BEACON_SEQ_MAX; i++) Seq[i] = BEACON_ID_NONE;
  SeqLen = 0u;
}

static bool PushBeaconIfNew(BeaconId_t id)
{
  if (SeqLen > 0u)
  {
    if (Seq[SeqLen - 1u] == id) return false;
  }

  if (SeqLen < BEACON_SEQ_MAX)
  {
    Seq[SeqLen++] = id;
  }
  else
  {
    /* shift left and append */
    for (uint8_t i = 1u; i < BEACON_SEQ_MAX; i++) Seq[i - 1u] = Seq[i];
    Seq[BEACON_SEQ_MAX - 1u] = id;
  }
  return true;
}

static bool MatchCycle(const BeaconId_t *cycle4, const BeaconId_t *obs, uint8_t nObs)
{
  /* allow any rotation offset of the 4-cycle */
  for (uint8_t offset = 0u; offset < 4u; offset++)
  {
    bool ok = true;
    for (uint8_t k = 0u; k < nObs; k++)
    {
      BeaconId_t expect = cycle4[(offset + k) & 0x03u];
      if (obs[k] != expect)
      {
        ok = false;
        break;
      }
    }
    if (ok) return true;
  }
  return false;
}

static Field_t DetermineFieldFromSequence(void)
{
  /*
    Green field CCW cycle: B L R G
    Blue  field CCW cycle: G R L B
  */
  if (SeqLen < 5u) return FIELD_UNKNOWN;

  BeaconId_t last5[5];
  for (uint8_t i = 0u; i < 5u; i++)
  {
    last5[i] = Seq[SeqLen - 5u + i];
  }

  const BeaconId_t greenCycle[4] = { BEACON_ID_B, BEACON_ID_L, BEACON_ID_R, BEACON_ID_G };
  const BeaconId_t blueCycle[4]  = { BEACON_ID_G, BEACON_ID_R, BEACON_ID_L, BEACON_ID_B };

  bool greenOK = MatchCycle(greenCycle, last5, 5u);
  bool blueOK  = MatchCycle(blueCycle,  last5, 5u);

  if (greenOK && !blueOK) return FIELD_GREEN;
  if (blueOK  && !greenOK) return FIELD_BLUE;

  return FIELD_UNKNOWN;
}

static void SquareUpOnT(ES_Event_t ThisEvent)
{
    switch (ThisEvent.EventType)
    {  
     
        case ES_T_DETECTED:
        {
            switch (ThisEvent.EventParam)
            {
                case FULL_T:
                    // creep forward
                    SetMotor1(-(int16_t)DUTY_TRANS_TAPE_DET);
                    SetMotor2(-(int16_t)DUTY_TRANS_TAPE_DET);
                    break;

                case RIGHT_CORNER:
                    // rotate LEFT (CCW)
                    SetMotor1((int16_t)DUTY_ROTATE);      // left backward
                    SetMotor2(-(int16_t)DUTY_ROTATE);     // right forward
                    break;

                case LEFT_CORNER:
                    // rotate RIGHT (CW)
                    SetMotor1(-(int16_t)DUTY_ROTATE);     // left forward
                    SetMotor2((int16_t)DUTY_ROTATE);      // right backward
                    break;

                default:
                    break;
            }
            break;
        }

        default:
            break;
    }
}