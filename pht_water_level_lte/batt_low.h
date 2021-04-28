#ifndef _BATT_LOW_H
#define _BATT_LOW_H
#include <stdint.h>
#include <stdbool.h>


extern uint16_t batt_low_threshold;
extern volatile uint16_t batt_low_interval;
extern uint16_t batt_low_current_level;
extern bool batt_low_most_recent_is_batt_low;


void batt_low_setup(void);
void batt_low_on_timer(void);
bool batt_low_send_if_needed(void);
bool batt_low_is_batt_low(bool is_modem_on);
uint16_t batt_low_get_level(void);


#endif // _BATT_LOW_H
