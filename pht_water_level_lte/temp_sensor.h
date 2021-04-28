#ifndef _TEMP_SENSOR_H
#define _TEMP_SENSOR_H
#include <stdint.h>


void temp_sensor_setup(void);
uint16_t temp_sensor_get_temp(void);


#endif // _TEMP_SENSOR_H
