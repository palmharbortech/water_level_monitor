#include "batt_low.h"
#include <avr/io.h>
#include "adc.h"


uint16_t batt_low_threshold = 0;
volatile uint16_t batt_low_interval = 0;
uint16_t batt_low_current_level = 0;
bool batt_low_most_recent_is_batt_low = false;

static struct {
    volatile bool timer_expired;
    uint16_t timer_cnt;
} m;


void batt_low_setup(void) {
    m.timer_expired = false;
    m.timer_cnt = 0;

    // PA3 (ADC0) is connected to VBATT
    DDRA &= ~(1 << PA3); // Configure PA3 as input

    // Disable Digital Input Register for ADC0
    DIDR0 |= (1 << ADC0D);
}


void batt_low_on_timer(void) {
    if (++m.timer_cnt == batt_low_interval) {
        m.timer_cnt = 0;
        m.timer_expired = true;
    }
}


bool batt_low_send_if_needed(void) {
    if (!m.timer_expired)
        return false;

    m.timer_expired = false;

    bool is_batt_low = batt_low_is_batt_low(false);
    if (is_batt_low != batt_low_most_recent_is_batt_low) {
        batt_low_most_recent_is_batt_low = is_batt_low;
        return true;
    }

    return false;
}


bool batt_low_is_batt_low(bool is_modem_on) {
    batt_low_current_level = batt_low_get_level();

    /*
    if (is_modem_on) {
        // The modem is on, which means the ADC will report lower than normal value (by as much as 45)
        batt_low_current_level += 45;
    }
    */

    /*
    // tmp
    const uint16_t address = 8;
    eeprom_write_word((void *)address, batt_low_current_level);
    // 0x03b2 -> 946
    // 0x039e -> 926
    // 0x03b5 -> 949

    const uint16_t address2 = 12;
    eeprom_write_word((void *)address2, 926);
    // end tmp
    */

    // When batt goes down to 1.908, it is low
    // ADC = (Vin * 1024) / Vref = (1.908 * 1024) / 3.4 = 574.6447
    // Vin = (ADC * Vref) / 1024
    /*
    bool is_batt_low = false;
    if (batt_low_current_level < batt_low_threshold)
        is_batt_low = true;

    return is_batt_low;
    */

    // tmp
    // 2019-03-12: For now, just always return false so the battery level is still read, but never goes below threshold
    return false;
    // end tmp

    // Hysteresis
    uint16_t batt_low_threshold_high = batt_low_threshold + 50;
    uint16_t batt_low_threshold_low = batt_low_threshold;

    if (batt_low_current_level < batt_low_threshold_low)
        return true;
    else if (batt_low_current_level > batt_low_threshold_high)
        return false;
    return batt_low_most_recent_is_batt_low;
}


uint16_t batt_low_get_level(void) {
    // In order to be able to use the ADC the Power Reduction bit, PRADC, in the Power Reduction Register must be disabled.
    // This is done by clearing the PRADC bit. See "PRR - Power Reduction Register" on page 38 for more details.
    //PRR &= ~(1 << PRADC); // don't need to do this, since it defaults to 0

    // The analog input channel is selected by writing to the MUX bits in ADMUX
    ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0); // 0000 ADC0 PA3

    return adc_do_conversion();
}
