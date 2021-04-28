#ifndef _ADC_H
#define _ADC_H
#include <stdint.h>


void adc_setup(void);
uint16_t adc_do_conversion(void);


#endif // _ADC_H
