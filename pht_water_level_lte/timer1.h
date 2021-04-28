#ifndef _TIMER1_H
#define _TIMER1_H
#include <stdint.h>
#include <stdbool.h>


enum {
    STATE_WRITE_CMD,
    STATE_WRITE_CMD_WAIT,
    STATE_WRITE_CMD_WAIT_OVER,
    STATE_WRITE_CMD_DELAY,
    STATE_WRITE_CMD_DELAY_OVER,
    STATE_WRITE_WAIT_FOR_UDP_PACKET,
    STATE_WRITE_IDLE,
    STATE_WRITE_BATT_LOW_DELAY,
    STATE_WRITE_BATT_LOW_DELAY_OVER,
    STATE_WRITE_WATER_LOW_DELAY,
    STATE_WRITE_WATER_LOW_DELAY_OVER,
    STATE_WRITE_TEMP_LOW_DELAY,
    STATE_WRITE_TEMP_LOW_DELAY_OVER,
    //STATE_WRITE_SET_CURRENT_TEMP_DELAY,
    //STATE_WRITE_SET_CURRENT_TEMP_DELAY_OVER,
    STATE_WRITE_SET_DAILY_DELAY,
    STATE_WRITE_SET_DAILY_DELAY_OVER,
    STATE_WRITE_HEARTBEAT_DELAY,
    STATE_WRITE_HEARTBEAT_DELAY_OVER,
    //STATE_WRITE_SET_GPS_DELAY,
    //STATE_WRITE_SET_GPS_DELAY_OVER,
    //STATE_WRITE_WAIT_FOR_GPS_RESPONSE,
};

extern volatile uint8_t state_write;
extern volatile uint8_t timer1_delay_100ms_cnt;
extern volatile bool timer1_temp_conversion_in_progress;


void timer1_setup(void);
void timer1_start(void);
void timer1_stop(void);
bool timer1_is_running(void);

#endif // _TIMER1_H
