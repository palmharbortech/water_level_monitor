#include "bootloader.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
//#define F_CPU 8000000UL // 8 MHz
//#define F_CPU 1000000UL // 1 MHz
#define F_CPU 1843200UL // 1.8432 MHz
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>


#define bl_i2c_scl_in_port PINC
#define bl_i2c_scl_out_port DDRC
#define bl_i2c_scl_pin PC2
#define bl_i2c_sda_in_port PINA
#define bl_i2c_sda_out_port DDRA
#define bl_i2c_sda_pin PA5
#define BL_I2CSPEED 100


// 0x0 bytes
// Return current level of SCL line, 0 or 1
static bool bl_read_SCL(void) {
    return bl_i2c_scl_in_port & (1 << bl_i2c_scl_pin);
}


// 0x0 bytes
// Return current level of SDA line, 0 or 1
static bool bl_read_SDA(void) {
    return bl_i2c_sda_in_port & (1 << bl_i2c_sda_pin);
}


// -Wl,--section-start=.bl_set_SCL=0x3c00
// 0x1e01 - 0x1e00 = 0x4 bytes
// Do not drive SCL (set pin high-impedance)
__attribute__ ((section (".bl_set_SCL"))) static void bl_set_SCL(void) {
    bl_i2c_scl_out_port &= ~(1 << bl_i2c_scl_pin); // Configure SCL as input
}


// 0x0 bytes
// Actively drive SCL signal low
static void bl_clear_SCL(void) {
    bl_i2c_scl_out_port |= (1 << bl_i2c_scl_pin); // Configure SCL as output
}


// -Wl,--section-start=.bl_set_SDA=0x3c04
// 0x1e03 - 0x1e02 = 0x4 bytes
// Do not drive SDA (set pin high-impedance)
__attribute__ ((section (".bl_set_SDA"))) static void bl_set_SDA(void) {
    bl_i2c_sda_out_port &= ~(1 << bl_i2c_sda_pin); // Configure SDA as input
}


// 0x0 bytes
// Actively drive SDA signal low
static void bl_clear_SDA(void) {
    bl_i2c_sda_out_port |= (1 << bl_i2c_sda_pin); // Configure SDA as output
}


// -Wl,--section-start=.bl_i2c_delay=0x3c08
// 0x1e14 - 0x1e04 = 0x22 bytes
__attribute__ ((section (".bl_i2c_delay"))) void bl_i2c_delay(void) {
    volatile int v = 0;
    for (uint8_t i = 0; i < BL_I2CSPEED / 2; ++i) {
        v;
    }
}

// -Wl,--section-start=.bl_i2c_start_cond=0x3c2a
// 0x1e2f - 0x1e15 = 0x36 bytes
__attribute__ ((section (".bl_i2c_start_cond"))) void bl_i2c_start_cond(bool *i2c_started) {
    if (*i2c_started) {
        // if started, do a restart condition
        // set SDA to 1
        bl_set_SDA();
        bl_i2c_delay();
        bl_set_SCL();
        while (bl_read_SCL() == 0) { // Clock stretching
            // You should add timeout to this loop
        }

        // Repeated start setup time, minimum 4.7us
        bl_i2c_delay();
    }

    if (bl_read_SDA() == 0) {
        //arbitration_lost();
        for (;;) {
        }
    }

    // SCL is high, set SDA from 1 to 0.
    bl_clear_SDA();
    bl_i2c_delay();
    bl_clear_SCL();
    *i2c_started = true;
}


// -Wl,--section-start=.bl_i2c_stop_cond=0x3c60
// 0x1e45 - 0x1e30 = 0x2c bytes
__attribute__ ((section (".bl_i2c_stop_cond"))) void bl_i2c_stop_cond(bool *i2c_started) {
    // set SDA to 0
    bl_clear_SDA();
    bl_i2c_delay();

    bl_set_SCL();
    // Clock stretching
    while (bl_read_SCL() == 0) {
        // add timeout to this loop.
    }

    // Stop bit setup time, minimum 4us
    bl_i2c_delay();

    // SCL is high, set SDA from 0 to 1
    bl_set_SDA();
    bl_i2c_delay();

    if (bl_read_SDA() == 0) {
        //arbitration_lost();
        for (;;) {
        }
    }

    *i2c_started = false;
}


// -Wl,--section-start=.bl_i2c_write_bit=0x3c8c
// 0x1e5c - 0x1e46 = 0x2e bytes
__attribute__ ((section (".bl_i2c_write_bit"))) void bl_i2c_write_bit(bool bit) {
    if (bit) {
        bl_set_SDA();
    } else {
        bl_clear_SDA();
    }

    // SDA change propagation delay
    bl_i2c_delay();

    // Set SCL high to indicate a new valid SDA value is available
    bl_set_SCL();

    // Wait for SDA value to be read by slave, minimum of 4us for standard mode
    bl_i2c_delay();

    while (bl_read_SCL() == 0) { // Clock stretching
        // You should add timeout to this loop
    }

    // SCL is high, now data is valid
    // If SDA is high, check that nobody else is driving SDA
    if (bit && (bl_read_SDA() == 0)) {
        //arbitration_lost();
        for (;;) {
        }
    }

    // Clear the SCL to low in preparation for next change
    bl_clear_SCL();
}


// -Wl,--section-start=.bl_i2c_read_bit=0x3cba
// 0x1e6c - 0x1e5d = 0x20 bytes
__attribute__ ((section (".bl_i2c_read_bit"))) bool bl_i2c_read_bit(void) {
    // Let the slave drive data
    bl_set_SDA();

    // Wait for SDA value to be written by slave, minimum of 4us for standard mode
    bl_i2c_delay();

    // Set SCL high to indicate a new valid SDA value is available
    bl_set_SCL();

    while (bl_read_SCL() == 0) { // Clock stretching
        // You should add timeout to this loop
    }

    // Wait for SDA value to be written by slave, minimum of 4us for standard mode
    bl_i2c_delay();

    // SCL is high, read out bit
    bool bit = bl_read_SDA();

    // Set SCL low in preparation for next operation
    bl_clear_SCL();

    return bit;
}


// -Wl,--section-start=.bl_i2c_write_byte=0x3cda
// 0x1e90 - 0x1e6d = 0x48 bytes
__attribute__ ((section (".bl_i2c_write_byte"))) bool bl_i2c_write_byte(bool *i2c_started, bool send_start, bool send_stop, uint8_t byte) {
    if (send_start)
        bl_i2c_start_cond(i2c_started);

    for (uint8_t bit = 0; bit < 8; ++bit) {
        bl_i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }

    bool nack = bl_i2c_read_bit();

    if (send_stop)
        bl_i2c_stop_cond(i2c_started);

    return nack;
}


// -Wl,--section-start=.bl_i2c_read_byte=0x3d22
// 0x1eb1 - 0x1e91 = 0x42 bytes
__attribute__ ((section (".bl_i2c_read_byte"))) uint8_t bl_i2c_read_byte(bool *i2c_started, bool nack, bool send_stop) {
    uint8_t byte = 0;

    for (uint8_t bit = 0; bit < 8; ++bit)
        byte = (byte << 1) | bl_i2c_read_bit();

    bl_i2c_write_bit(nack);

    if (send_stop)
        bl_i2c_stop_cond(i2c_started);

    return byte;
}


// Read data from external eeprom and self-program except don't overwrite the bootloader.
// The bootloader itself will never be overwritten via remote firmware upgrade, so it needs to be bug free.
// Bootloader should be at the top, so it should start at 16384 - 512 = 15872 = 0x3e00
// But other functions should start at 16384 - 512 - 512 = 15360 = 0x3c00
// -Wl,--section-start=.bl_update_firmware_from_eeprom=0x3e00
// 0x1fb5 - 0x1f00 = 0x9a * 2 = 0x16c bytes
__attribute__ ((section (".bl_update_firmware_from_eeprom"))) void bl_update_firmware_from_eeprom(void) {
    bool i2c_started = false;

    // Erase all pages using 4-page erase command (~500ms)
    // 1 page is 32 bytes, 512 total pages, so 0x3E00 is the top 16 pages
    // 1 page is 32 bytes, 512 total pages, so 0x3c00 is the top 32 pages
    for (uint16_t page = 0; page < (512 - 32); page += 4) {
        uint16_t page_address = page * 32; // page_address is a byte address
        boot_page_erase_safe(page_address);
    }

    boot_spm_busy_wait();
    eeprom_busy_wait();

    // Start reading external eeprom at address 0x00
    // ST and SAD+W
    if (bl_i2c_write_byte(&i2c_started, true, false, 0xa2)) { // 1010 0010
        return; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte1
    if (bl_i2c_write_byte(&i2c_started, false, false, 0x00)) {
        return; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte2
    if (bl_i2c_write_byte(&i2c_started, false, false, 0x00)) {
        return; // Slave failed to ACK
    }

    _delay_ms(2);

    // SR and SAD+R
    if (bl_i2c_write_byte(&i2c_started, true, false, 0xa3)) { // 1010 0011
        return; // Slave failed to ACK
    }

    _delay_ms(3);

    // This seems to work, but I don't fully understand why, need more testing to
    // find that the boundary is where I'm expecting it to be.
    for (uint16_t page = 0; page < (1024 - 64); ++page) {
        uint16_t page_address = page * 16;

        // For each word in the page
        for (uint16_t word_address = 0; word_address < 32 / 2; word_address += 2) {
            // Read the word from external_eeprom
            uint8_t data_byte1 = bl_i2c_read_byte(&i2c_started, false, false);
            uint8_t data_byte2 = bl_i2c_read_byte(&i2c_started, false, false);

            // Set up little-endian word
            uint16_t word_data_le = data_byte1;
            word_data_le += (data_byte2) << 8;

            uint16_t byte_address = page_address + word_address;
            boot_page_fill(byte_address, word_data_le);
        }

        boot_page_write_safe(page_address);

        eeprom_busy_wait();
        boot_spm_busy_wait();
    }

    bl_i2c_read_byte(&i2c_started, true, true);

    wdt_enable(WDTO_8S);
    for (;;) {
    }
}
