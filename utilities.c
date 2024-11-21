#include "utilities.h"

#define FOSC                    10000000
//Timer2 tick period with 1:1 prescaler (with Fosc/4 as forced input)
#define TIMR2_TICK_PERIOD_NS    (400)

//this delay uses TMR2
void delay_us(uint32_t delay_us) {
    //reset timer
    TMR2 = 0x00;
    PR2 = 0xFF;
    TMR2IF = 0;
    TMR2ON = 1;
    
    //counts the timer2 overflows
    uint32_t overflow = 0;
    uint32_t target_ticks = (delay_us*1000)/(TIMR2_TICK_PERIOD_NS);
    
    uint32_t target_overflow = target_ticks/0xFF;
    uint8_t target_timer = target_ticks - target_overflow*0xFF;
    
    while(overflow < target_overflow || TMR2 < target_timer) {
        if(TMR2IF == 1) {
            overflow++;
            TMR2IF = 0;
        }
    }
    TMR2ON = 0;
    return;
}
