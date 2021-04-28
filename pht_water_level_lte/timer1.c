#include "timer1.h"
#include <avr/io.h>
#include <avr/interrupt.h>
//#define F_CPU 8000000UL // 8 MHz
//#define F_CPU 1000000UL // 1 MHz
#define F_CPU 1843200UL // 1.8432 MHz
#include <util/delay.h>


volatile uint8_t state_write;
volatile uint8_t timer1_delay_100ms_cnt;
volatile bool timer1_temp_conversion_in_progress = false;


#define TIMER1_100MS 0xa600 // 100.0ms
//#define TIMER1_100MS 0xa580 // 100.6ms
//#define TIMER1_100MS 0xa500 // 101.1ms
//#define TIMER1_100MS 0x9f00 // 107.8ms
//#define TIMER1_100MS 0x9e01 // 108.9ms


void timer1_setup(void) {
    //TCNT1 = TIMER1_100MS; // load the timer counter value
    TIMSK |= (1 << TOIE1); // Timer/Counter1, Overflow Interrupt Enable
    //TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // Clock Select (starts the timer)

    //sei(); // Enable global interrupts
}


void timer1_start(void) {
    TCNT1 = TIMER1_100MS; // load the timer counter value
#if F_CPU == 1000000UL
    TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // Clock Select (starts the timer)
#elif F_CPU == 1843200UL
    TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // Clock Select (starts the timer)
#elif F_CPU == 8000000UL
    TCCR1B = (0 << CS12) | (1 << CS11) | (1 << CS10); // Clock Select (starts the timer)
#endif
    //sei(); // enable interrupts
}


/*
void timer1_start_50ms(void) {
    TCNT1 = TIMER1_50MS; // load the timer counter value
    TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // Clock Select (starts the timer)
    //sei(); // enable interrupts
}
*/


void timer1_stop(void) {
    TCCR1B = (0 << CS12) | (0 << CS11) | (0 << CS10); // Clock Select (stops the timer)
    //cli(); // disable interrupts
}


bool timer1_is_running(void) {
    return (TCCR1B & (1 << CS11));
}


// Timer1 Overflow interrupt
ISR(TIMER1_OVF_vect) {
    if (timer1_temp_conversion_in_progress) {
        timer1_temp_conversion_in_progress = false;
        return;
    }

    switch (state_write) {
    case STATE_WRITE_CMD_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_CMD_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
    case STATE_WRITE_BATT_LOW_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_BATT_LOW_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
    case STATE_WRITE_WATER_LOW_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_WATER_LOW_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
    case STATE_WRITE_TEMP_LOW_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_TEMP_LOW_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
        /*
    case STATE_WRITE_SET_CURRENT_TEMP_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_SET_CURRENT_TEMP_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
        */
    case STATE_WRITE_SET_DAILY_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_SET_DAILY_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
    case STATE_WRITE_HEARTBEAT_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_HEARTBEAT_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
        /*
    case STATE_WRITE_SET_GPS_DELAY:
        if (timer1_delay_100ms_cnt <= 1) {
            state_write = STATE_WRITE_SET_GPS_DELAY_OVER;
        } else {
            --timer1_delay_100ms_cnt;
            TCNT1 = TIMER1_100MS; // load the timer counter value
        }
        break;
        */
    default:
        break;
    }
}
