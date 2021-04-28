#ifndef _WATER_LOW_H
#define _WATER_LOW_H
#include <stdint.h>
#include <stdbool.h>


extern uint16_t water_low_threshold;
extern volatile uint16_t water_low_interval;
extern uint16_t water_low_current_level;
extern bool water_low_most_recent_is_water_low;


void water_low_setup(void);
void water_low_on_timer(void);
bool water_low_send_if_needed(void);
bool water_low_is_water_low(void);
uint16_t water_low_get_level(void);


#endif // _WATER_LOW_H
