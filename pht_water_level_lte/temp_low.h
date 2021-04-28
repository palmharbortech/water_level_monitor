#ifndef _TEMP_LOW_H
#define _TEMP_LOW_H
#include <stdint.h>
#include <stdbool.h>


extern uint16_t temp_low_threshold;
extern volatile uint16_t temp_low_interval;
extern uint16_t temp_low_current_level;
extern bool temp_low_most_recent_is_temp_low;


void temp_low_setup(void);
void temp_low_on_timer(void);
bool temp_low_send_if_needed(void);
bool temp_low_is_temp_low(void);
uint16_t temp_low_get_level(void);


#endif // _TEMP_LOW_H
