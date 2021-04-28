#include "temp_low.h"
#include <avr/io.h>
#include "temp_sensor.h"


uint16_t temp_low_threshold = 0;
volatile uint16_t temp_low_interval = 0;
uint16_t temp_low_current_level = 0;
bool temp_low_most_recent_is_temp_low = false;

static struct {
    volatile bool timer_expired;
    uint16_t timer_cnt;
} m;


void temp_low_setup(void) {
    m.timer_expired = false;
    m.timer_cnt = 0;
}


void temp_low_on_timer(void) {
    if (++m.timer_cnt == temp_low_interval) {
        m.timer_cnt = 0;
        m.timer_expired = true;
    }
}


bool temp_low_send_if_needed(void) {
    if (!m.timer_expired)
        return false;

    m.timer_expired = false;

    bool is_temp_low = temp_low_is_temp_low();
    if (is_temp_low != temp_low_most_recent_is_temp_low) {
        temp_low_most_recent_is_temp_low = is_temp_low;
        return true;
    }

    return false;
}


bool temp_low_is_temp_low(void) {
    temp_low_current_level = temp_low_get_level();

    // Calibration offset
    //temp_low_current_level -= 6; // 6 * 0.0625 = 0.675F
    temp_low_current_level -= 7;   // 7 * 0.0625 = 0.7875F
    //temp_low_current_level -= 8; // 8 * 0.0625 = 0.9F

    // Hysteresis
    // 950 -> 48.875F
    // 994 -> 53.825F
    uint16_t temp_low_threshold_high = temp_low_threshold + 44; // 44 * 0.0625 = 4.95F
    uint16_t temp_low_threshold_low = temp_low_threshold;

    if (temp_low_current_level < temp_low_threshold_low)
        return true;
    else if (temp_low_current_level > temp_low_threshold_high)
        return false;
    return temp_low_most_recent_is_temp_low;
}


uint16_t temp_low_get_level(void) {
    return temp_sensor_get_temp();
}
