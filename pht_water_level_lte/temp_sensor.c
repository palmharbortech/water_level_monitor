#include "temp_sensor.h"
#include <avr/io.h>
//#define F_CPU 8000000UL // 8 MHz
//#define F_CPU 1000000UL // 1 MHz
#define F_CPU 1843200UL // 1.8432 MHz
#include <util/delay.h>
#include "timer1.h"


void temp_sensor_setup(void) {
    PORTA |= (1 << PA4); // LMT01_VP up
    DDRA &= ~(1 << PA4); // Configure LMT01_VP as input

    // Configure the port pin as input with the internal pullup switched off to avoid the digital
    // port function from interfering with the function of the Analog Comparator.
    //DDRA &= ~(1 << PA1); // Configure LMT01_VN as input (this is the default)

    // To switch the pull-up resistor off, PORTxn has to be written logic zero or the pin has to be configured as an output pin.
    //PORTA &= ~(1 << PA1); // LMT01_VN down (this is the default)

    // Need a reference voltage of 513mV (or thereabouts), r1=91k r2=16k 0.508mV
    //DDRA &= ~(1 << PA2); // Configure VCC05 as input (this is the default)

    // To switch the pull-up resistor off, PORTxn has to be written logic zero or the pin has to be configured as an output pin.
    //PORTA &= ~(1 << PA2); // VCC05 down (this is the default)

    // When this bit is written logic one, the digital input buffer on the AIN1/0 pin is disabled
    DIDR0 |= (1 << AIN1D) | (1 << AIN0D);

    ACSRA |= (1 << ACIS1) | (1 << ACIS0); // Comparator Interrupt on Rising Output Edge

    ACSRB |= (1 << HLEV); // set hysteresis level to 50mV
    ACSRB |= (1 << HSEL); // enable hysteresis of the analog comparator
}


uint16_t temp_sensor_get_temp(void) {
    // Must turn temp sensor on prior to enabling interrupts, because LMT01_VN spikes up to 2.85v for about 30us
    DDRA |= (1 << PA4); // Configure LMT01_VP as output

    _delay_ms(1);

    // Clear any existing interrupt
    // ACI is cleared by writing a logic one to the flag
    ACSRA |= (1 << ACI); // Analog Comparator Interrupt Flag

    // This triggers the calling of the isr, but calling the isr is too slow, so need to examine ACI in a fast loop instead
    //ACSRA |= (1 << ACIE); // Analog Comparator Interrupt Enable

    // TODO: timer1_stop()?
    timer1_temp_conversion_in_progress = true;
    timer1_start(); // 100ms

    // After the LMT01 is powered up, it transmits a very low current of 34 µA for less than 54 ms
    // while the part executes a temperature to digital conversion

    // Start counting rising edges on LMT01_VN, stop counting after 50ms from time of first rising edge
    // We will always get at least 1 rising edge

    // Measuring LMT01_VN
    // 223mV (logic low while idling)
    // 254mV (logic low while sending data)
    // 798mV (logic high while sending data)

    uint16_t count = 0;
    while (timer1_temp_conversion_in_progress) {
        // Needed to unroll this loop because it wasn't quite fast enought at 1MHz (although it worked fine at 8MHz)
        // 2 works, but 1 doesn't
        if (ACSRA & (1 << ACI)) {
            // ACI is cleared by writing a logic one to the flag
            ACSRA |= (1 << ACI);

            ++count;
        }

        if (ACSRA & (1 << ACI)) {
            // ACI is cleared by writing a logic one to the flag
            ACSRA |= (1 << ACI);

            ++count;
        }
    }

    timer1_stop();

    //ACSRA &= ~(1 << ACIE); // Analog Comparator Interrupt Disable
    DDRA &= ~(1 << PA4); // Configure LMT01_VP as input

    // tmp
    //const uint16_t address = 8;
    //eeprom_write_word((void *)address, count);
    // end tmp

    // uint8_t *temperature_high, uint8_t *temperature_low
    //*temperature_high = HI_BYTE(count);
    //*temperature_low = LO_BYTE(count);

    // Take care to ensure that a minimum power-down wait time of 50 ms is used before the device is turned on again.
    _delay_ms(50);

    return count;
}
