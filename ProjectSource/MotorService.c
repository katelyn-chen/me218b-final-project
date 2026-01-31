/*----------------------------- Include Files -----------------------------*/
// This module
#include "MotorService.h"

// Hardware
#include <xc.h>
#include <sys/attribs.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include "ADService.h"

/*----------------------------- Module Defines ----------------------------*/
// Period = Target / (PBClk * Prescaler) - 1
#define PWM_PERIOD 49999 // 200Hz PWM for PBClk = 20MHz;
//#define PWM_PERIOD 39999 // 250Hz, 1:2; 500Hz, 1:1
//#define PWM_PERIOD 19999 // 1000Hz, 1:1
//#define PWM_PERIOD 9999 // 2000Hz, 1:1
// #define PWM_PERIOD 1999 // 10000Hz, 1:1

#define PRESCALE 2.0
#define PERCENT 1.0
#define MAX_RPM 150.0 // Approximate for large motor

// Pin Definitions
#define IO_LINE                 LATBbits.LATB8
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void InitTimer2();
void InitTimer4();
void InitTimer3();
void InitPins();

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static volatile uint32_t LastCapture=0;
static volatile uint32_t Period=0;
static bool NewEdge = false;
static volatile uint32_t RolloverCount=0;
static volatile float targetRPM=0;

static float Kp = 0.5; 
static float Ki = 0.2;
static float accumulatedError = 0;

/*------------------------------ Module Code ------------------------------*/

bool InitMotorService(uint8_t Priority)
{    
    MyPriority = Priority;
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_INIT;
    
    InitPins();
    InitTimer2();
    InitTimer3();
    InitTimer4();

    // (configure output compare)
    OC1CONbits.ON = 0;
    OC1CONbits.OCTSEL = 0;
    OC1R = 0;
    OC1RS = 0; // initialize duty cycle to 0%
    OC1CONbits.OCM = 0b110; // PWM mode on Timer2
    
    // (configure input capture)
    INTCONbits.MVEC=1;
    IC2R = 0b0011; // map IC1 to RB10
    IC2CONbits.ON = 0;
    IC2CONbits.C32 = 0;
    IC2CONbits.ICTMR = 0; // use Timer3
    IC2CONbits.ICM = 0b011; // capture every rising edge on Timer3
    IC2CONbits.ICI = 0b000;
    IPC2bits.IC2IP = 7;
    IEC0SET = _IEC0_IC2IE_MASK; // enables IC  for timer 3
    IFS0CLR = _IFS0_IC2IF_MASK; // clear interrupt flag

    

    IC2CONbits.ON = 1;
    // start timers 
    OC1CONbits.ON = 1; // turn on OC1 module
    T2CONbits.ON = 1; // start Timer2
    T3CONbits.ON = 1; // start Timer3
    // T4CONbits.ON = 1; // start Timer4
    
    __builtin_enable_interrupts();
    
    if (ES_PostToService(MyPriority, ThisEvent) == true) {
        return true;
    } else {
        return false;
    }
}

bool PostMotorService(ES_Event_t ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;
    switch (ThisEvent.EventType) {
        case ES_INIT:
        {
          DB_printf("\rES_INIT received in MotorService\n");
        }
        break;

        case ES_TIMEOUT:
        {
            if (ThisEvent.EventParam == MotorTimer) {
                // Force direction to Forward
                LATBbits.LATB2 = 1;
                LATBbits.LATB3 = 0;
                
                // --- MODIFICATION FOR TESTING ---
                // Set a constant 50% Duty Cycle step
                OC1RS = PR2 / 2; 
                
                // Print status just to confirm code is running
                DB_printf("Testing Mode: 10Hz Step Active\n");
                
                ES_Timer_InitTimer(MotorTimer, 500); // Slow down the print rate
//                 if (PORTBbits.RB8 == 1) {
//                     // set motor direction forward
//                     LATBbits.LATB2 = 1;
//                     LATBbits.LATB3 = 0;
//                 } else {
//                     // set motor direction reverse
//                     LATBbits.LATB2 = 0;
//                     LATBbits.LATB3 = 1;
//                 }
              
//                 // query ADService for next time interval
//                 int32_t PotValue = GetPotValue();
// //                DB_printf("PotValue: %d \n:", PotValue);
//                 // set OC1RS for new duty cycle
//                 OC1RS = PotValue * PWM_PERIOD * PERCENT / 1023; // scale 0-1023 to 0-PWM_PERIOD
//                 targetRPM = (float)PotValue * MAX_RPM / 1023.0;
//                 DB_printf("Duty: %d \n", OC1RS);
//                 DB_printf("Percent: %d \n", PERCENT);
// //                DB_printf("LastCapture: %d\n", LastCapture);
// //                DB_printf("RolloverCount: %d\n", RolloverCount);
//                 if (NewEdge == true) {
//                     uint32_t currentPeriod = Period;
//                     DB_printf("Current Period: %d\n", currentPeriod);
//                     NewEdge = false;
//                  }
//                 // calculate RPM
//                 uint32_t rpm = 0;
//                 if (Period != 0) {
//                     rpm = (60 * (20000000 / Period) / (512 * 3));
//                 }
//                 DB_printf("rpm: %d\n", rpm);
//                 ES_Timer_InitTimer(MotorTimer, 100);
            }
        }
        break;
    }
    return ReturnEvent;
}

void InitPins() {
    // configure h-bridge pins as digital outputs (RB2, RB3)
    TRISBbits.TRISB2 = 0; // RB2 output
    ANSELBbits.ANSB2 = 0;
    TRISBbits.TRISB3 = 0; // RB3 output
    ANSELBbits.ANSB3 = 0;
    RPB3Rbits.RPB3R = 0b0101;

    TRISBbits.TRISB8 = 1; // RB8 digital input
    
    TRISBbits.TRISB10 = 1; // RB10, encoder input

    // configure pins for 74ACT244 (RB4,5,9,11,12,13,14,15)
    TRISBbits.TRISB4 = 0; // RB4 output
    TRISBbits.TRISB5 = 0; // RB5 output
    TRISBbits.TRISB9 = 0; // RB9 output
    TRISBbits.TRISB11 = 0; // RB11 output
    TRISBbits.TRISB12 = 0; // RB12 output
    TRISBbits.TRISB13 = 0; // RB13 output
    TRISBbits.TRISB14 = 0; // RB14 output
    TRISBbits.TRISB15 = 0; // RB15 output
    ANSELBbits.ANSB12 = 0;
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB15 = 0;
    
}
void InitTimer2() {
    // (configure Timer 2 for PWM)
    T2CONbits.ON = 0; // turn off Timer2
    T2CONbits.TCS = 0; // select internal PBCLK source
    // T2CONbits.TCKPS = 0b010; // select prescaler 1:2        
    // T2CONbits.TCKPS = 0b000; // prescaler 1:1        

    // measuring time constant
    T2CONbits.TCKPS = 0b110; // Change prescaler to 1:64 for a slow clock
    PR2 = 31249;             // Set for 10Hz (20MHz / 64 / 10Hz - 1)

    TMR2 = 0; // clear timer 2 counter
    // PR2 = PWM_PERIOD; // set period for 200Hz PWM
    IFS0CLR = _IFS0_T2IF_MASK; // clear interrupt flag
}

void InitTimer3() {
    // (configure Timer 3 for input capture)
    T3CONbits.ON = 0; // turn off Timer3
    T3CONbits.TCS = 0;
    T3CONbits.TCKPS = 0b010; // select prescaler 1:2        
    // T3CONbits.TCKPS = 0b000; // prescaler 1:1        
    TMR3 = 0; // clear timer 3 counter
    PR3 = 0xFFFF;
    IPC3bits.T3IP = 6;
    IEC0SET = _IEC0_T3IE_MASK; // enables IC  for timer 3
    IFS0CLR = _IFS0_T3IF_MASK; // clear interrupt flag
}

void InitTimer4() {
    // (configure Timer 4 for control loop timing)
    T4CONbits.ON = 0; // turn off Timer4
    T4CONbits.TCS = 0;
    T4CONbits.TCKPS = 0b000; // prescaler 1:1        
    TMR4 = 0; // clear timer 4 counter
    PR4 = 19999; // 200MHz control loop with 1:2 prescaler
    IPC4bits.T4IP = 5;
    IEC0SET = _IEC0_T4IE_MASK; // enables IC  for timer 4
    IFS0CLR = _IFS0_T4IF_MASK; // clear interrupt flag
    
}

 void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SOFT) IC2Handler(void)
 {
     uint32_t CaptureTime = IC2BUF; // read the captured time
     IFS0CLR = _IFS0_IC2IF_MASK; // clear the interrupt flag
     if(IFS0bits.T3IF && (CaptureTime < 0x8000)) { 
         RolloverCount++;
         IFS0CLR = _IFS0_T3IF_MASK;
     }
     uint32_t CurrentCapture = (RolloverCount << 16) | CaptureTime;
     Period = CurrentCapture - LastCapture;
     LastCapture = CurrentCapture;
     NewEdge = true;
     return;
 }
 
 // separate isr thing for rollover w/ IPL6SOFT
 void __ISR(_TIMER_3_VECTOR, IPL6SOFT) TIMER2Handler(void) {
     __builtin_disable_interrupts();
     if (IFS0bits.T3IF) {
         RolloverCount++;
         IFS0CLR = _IFS0_T3IF_MASK;
     }
     __builtin_enable_interrupts();
 }

 void __ISR(_TIMER_4_VECTOR, IPL5SOFT) TIMER4Handler(void) {
    __builtin_disable_interrupts();
    IO_LINE = 1; 

    if (Period != 0) {
        // Calculate Actual RPM (Large Motor: 512 PPR, 5.9:1 Gearbox)
        float actualRPM = (60.0 * (20000000.0 / Period)) / (512.0 * 5.9);
        
        // Calculate Speed Error
        float error = targetRPM - actualRPM;
        
        // Proportional + Integral Control Law
        accumulatedError += error;
        float pTerm = Kp * error;
        float iTerm = Ki * accumulatedError;
        float requestedDuty = pTerm + iTerm;
        
        // Anti-Windup & Saturation Logic
        // If the calculated duty exceeds hardware limits, stop integrating
        if (requestedDuty > 100.0) {
            requestedDuty = 100.0;
            accumulatedError -= error; // Anti-windup
        } else if (requestedDuty < 0.0) {
            requestedDuty = 0.0;
            accumulatedError -= error; // Anti-windup
        }
        
        // Set new duty cycle
        OC1RS = (uint32_t)((requestedDuty / 100.0) * PWM_PERIOD);
    }
    
    // Lower I/O line 
    IO_LINE = 0; 
    IFS0CLR = _IFS0_T4IF_MASK;
    __builtin_enable_interrupts();
}