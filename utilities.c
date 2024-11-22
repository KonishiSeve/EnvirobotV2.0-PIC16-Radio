#include "utilities.h"

#define FOSC                    10000000
//Timer2 tick period with 1:1 prescaler (with Fosc/4 as forced input)
#define TIMR2_TICK_PERIOD_NS    (400)

// ====== LED ===== //
void led_init(void) {
    TRISA5 = 0;
    RA5 = 0;
}

void led_state(uint8_t state) {
    RA5 = state;
}

// ===== TIME ===== //
//this delay uses TIMER2
void delay_us(uint32_t delay_us) {
    TMR2 = 0x00;    //reset the timer
    PR2 = 0xFF;     //compare to max value
    TMR2IF = 0;     //reset interrupt/overflow flag
    TMR2ON = 1;     //start the timer
    
    uint32_t overflow = 0;  //counts the timer2 overflows
    //convert delay to timer ticks
    uint32_t target_ticks = (delay_us*1000)/(TIMR2_TICK_PERIOD_NS);
    //convert target ticks to number of overflows + timer value
    uint32_t target_overflow = target_ticks/0xFF;
    uint8_t target_timer = target_ticks - target_overflow*0xFF;
    
    while(overflow < target_overflow || TMR2 < target_timer) {
        //increase the counter when interrupt flag activates (because of an overflow)
        if(TMR2IF == 1) {
            overflow++;
            TMR2IF = 0;
        }
    }
    TMR2ON = 0; //disable timer
    return;
}
