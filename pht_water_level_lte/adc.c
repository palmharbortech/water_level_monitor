#include "adc.h"
#include <avr/io.h>


void adc_setup(void) {
    // No interrupts (should be already off)

    // The ADC voltage reference is selected by writing the REFS[1:0] bits in the ADMUX register
    //ADMUX &= ~(1 << REFS1) & ~(1 << REFS0); // VCC (3.4v) used as analog reference, disconnected from PA0 (AREF)

    // The analog input channel is selected by writing to the MUX bits in ADMUX
    //ADMUX &= ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0); // 0000 ADC0 PA3

    // ADC Prescaler Select Bits 011 -> 8 so 1MHz / 8 = 125kHz (needs to be under 200kHz for max resolution)
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
}


uint16_t adc_do_conversion(void) {
    // The ADC is enabled by setting the ADC Enable bit, ADEN in ADCSRA
    ADCSRA |= (1 << ADEN);

    // A single conversion is started by writing a logical one to the ADC Start Conversion bit, ADSC
    ADCSRA |= (1 << ADSC);

    // This bit stays high as long as the conversion is in progress and will be cleared by hardware when the conversion is completed
    // Measured at 206us, which is 1 / 125kHz = 8us * 25 ADC clock cycles = 200us
    while (ADCSRA & (1 << ADSC));

    // The ADC generates a 10-bit result which is presented in the ADC Data Registers, ADCH and ADCL
    // ADCL must be read first, then ADCH
    uint16_t level = ADCL;
    level |= (uint16_t)(ADCH << 8);

    // Turn ADC off
    ADCSRA &= ~(1 << ADEN);
    return level;
}
