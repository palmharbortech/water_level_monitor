#include "water_low.h"
#include <avr/io.h>
//#define F_CPU 8000000UL // 8 MHz
//#define F_CPU 1000000UL // 1 MHz
#define F_CPU 1843200UL // 1.8432 MHz
#include <util/delay.h>
#include "adc.h"


uint16_t water_low_threshold = 0;
volatile uint16_t water_low_interval = 0;
uint16_t water_low_current_level = 0;
bool water_low_most_recent_is_water_low = false;

static struct {
    volatile bool timer_expired;
    uint16_t timer_cnt;
} m;


void water_low_setup(void) {
    m.timer_expired = false;
    m.timer_cnt = 0;

    PORTB &= ~(1 << PB1); // ETRODE_SUPPLY down
    DDRB |= (1 << PB1); // Configure ETRODE_SUPPLY as output

    PORTA &= ~(1 << PA6); // ETRODE_TEST down
    DDRA |= (1 << PA6); // Configure ETRODE_TEST as output

    // PA6 (ADC3) is connected to ETRODE_TEST
    //DDRA &= ~(1 << PA6); // Configure PA6 as input

    // Disable Digital Input Register for ADC3
    DIDR0 |= (1 << ADC3D);
}


void water_low_on_timer(void) {
    if (++m.timer_cnt == water_low_interval) {
        m.timer_cnt = 0;
        m.timer_expired = true;
    }
}


bool water_low_send_if_needed(void) {
    if (!m.timer_expired)
        return false;

    m.timer_expired = false;

    bool is_water_low = water_low_is_water_low();
    if (is_water_low != water_low_most_recent_is_water_low) {
        water_low_most_recent_is_water_low = is_water_low;
        return true;
    }

    return false;
}


bool water_low_is_water_low(void) {
    water_low_current_level = water_low_get_level();

    // Hysteresis
    uint16_t water_low_threshold_high = water_low_threshold + 80;
    uint16_t water_low_threshold_low = water_low_threshold;

    if (water_low_current_level < water_low_threshold_low) {
        // ETRODE_TEST is being pulled low because water is present, so return false because 'water is not low'
        return false;
    } else if (water_low_current_level > water_low_threshold_high) {
        // ETRODE_TEST is being pulled high because water is not present, so return true because 'water is low'
        return true;
    }
    return water_low_most_recent_is_water_low;
}


static void electrode_level_begin(void) {
    DDRA &= ~(1 << PA6); // Configure ETRODE_TEST as input

    PORTB |= (1 << PB1); // ETRODE_SUPPLY up
}


static void electrode_level_end(void) {
    PORTB &= ~(1 << PB1); // ETRODE_SUPPLY down

    //PORTA &= ~(1 << PA6); // ETRODE_TEST down
    DDRA |= (1 << PA6); // Configure ETRODE_TEST as output
}


uint16_t water_low_get_level(void) {
    electrode_level_begin();

    // Seems to take about 200us for ETRODE_TEST to be pulled up by R5 (5.1MOhms)
    _delay_ms(2);

    // The analog input channel is selected by writing to the MUX bits in ADMUX
    ADMUX &= ~(1 << MUX3) & ~(1 << MUX2); // 0011 ADC3 PA6
    ADMUX |= (1 << MUX1) | (1 << MUX0);

    uint16_t level = adc_do_conversion();
    electrode_level_end();
    return level;
}
