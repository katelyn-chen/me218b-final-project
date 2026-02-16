/*----------------------------- Include Files -----------------------------*/
// This module
#include "ADService.h"

// Hardware
#include <xc.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h" 


/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static int32_t PotValue = 0;

/*------------------------------ Module Code ------------------------------*/


bool InitADService(uint8_t Priority)
{
    DB_printf("ADService init\n");
    ADC_ConfigAutoScan(BIT0HI); // Configure AN0 (Pin 2) for A/D input
    MyPriority = Priority;
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == true) {
        return true;
    } else {
        return false;
    }

}

bool PostADService(ES_Event_t ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunADService(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;
    switch (ThisEvent.EventType) {
        case ES_INIT:
        {
           ES_Timer_InitTimer(ADTimer, 100); // 10Hz
           ES_Timer_InitTimer(MotorTimer, 100); // 10Hz
           DB_printf("Timers initialized\n");
        }
        break;

        case ES_TIMEOUT:
        {
            if (ThisEvent.EventParam == ADTimer) {
                uint32_t ADValue[1]; // read A/D value
                ADC_MultiRead(ADValue);
//                DB_printf("ADValue: %d\r\n", ADValue[0]);
                PotValue = ADValue[0];
                ES_Timer_InitTimer(ADTimer, 100);
            }
        }
        break;
    }
    return ReturnEvent;
}

uint32_t GetPotValue(void)
{
    return PotValue;
}