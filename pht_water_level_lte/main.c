/*
 * pht_water_level_lte.c
 *
 * Created: 2/28/2019 2:55:24 PM
 * Author : bshiflet
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/power.h>
//#define F_CPU 8000000UL // 8 MHz
//#define F_CPU 1000000UL // 1 MHz
#define F_CPU 1843200UL // 1.8432 MHz
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


#define pht_i2c_scl_in_port PINC
#define pht_i2c_scl_out_port DDRC
#define pht_i2c_scl_pin PC2
#define pht_i2c_sda_in_port PINA
#define pht_i2c_sda_out_port DDRA
#define pht_i2c_sda_pin PA5
#include "../../../lib/pht_i2c.h"
#include "../../../lib/pht_fathom_payload.h"

#include <avr/pgmspace.h> // strlen_P

#include "adc.h"
#include "batt_low.h"
#include "bootloader.h"
#include "temp_low.h"
#include "temp_sensor.h"
#include "timer1.h"
#include "water_low.h"

#define ARRLEN(x) (sizeof(x) / sizeof(x[0]))
#define HI_BYTE(x) ((x) >> 8)
#define LO_BYTE(x) ((x) & 0xff)


const uint16_t firmware_version = 12; // this could go in eeprom, but try leaving it here
const uint8_t protocol_version = 1;

const uint16_t fathom_id_base_address = 0;
uint16_t fathom_id = 0; // this is read from eeprom upon startup

uint8_t signal_strength_rsrq;
uint8_t signal_strength_rsrp;
//char network_name[16];
Pht_fathom_payload fathom_payload;
uint8_t firmware_page_copy[64];
uint8_t firmware_page_num_last_received;

uint8_t read_buf[110];

char data_for_sendto_cmd[38];
//uint8_t data_for_sendto_data_cmd[10];
uint8_t data_for_sendto_data_cmd[14];
uint8_t data_for_sendto_data_cmd_len = 0;
char data_for_receive_from_cmd[16];
char data_for_set_apn_cmd[33];
//uint16_t power_down_cnt_set_current_temp = 0;
uint16_t power_down_cnt_set_daily_temp = 0;
uint16_t power_down_cnt_heartbeat = 0;
//bool power_down_cnt_set_gps = false;
//uint16_t set_current_temp_interval = 0;
uint16_t heartbeat_interval = 0;

//uint8_t set_current_temp_temperature_high = 0;
//uint8_t set_current_temp_temperature_low = 0;

uint16_t set_daily_temp_daily_high = 0;
uint16_t set_daily_temp_daily_low = UINT16_MAX;

//bool has_first_day_elasped = false;
uint16_t fathom_days_offset = 0;
uint16_t fathom_time_of_day = 0; // in 2s steps
uint16_t set_daily_temp_last_sent = 0; // in 2s steps
uint16_t postpone_set_daily_by;

uint32_t wait_for_udp_packet_cnt = 0;
//uint32_t wait_for_udp_packet_threshold = 0x00010000; // 2.6s (testing)
//uint32_t wait_for_udp_packet_threshold = 0x00040000; // 10.25s + 5.6s
//uint32_t wait_for_udp_packet_threshold = 0x00048000; // 11.52s + 5.6s
//uint32_t wait_for_udp_packet_threshold = 0x00070000; // 17.92s
uint32_t wait_for_udp_packet_threshold = 0x00080000; // 20.52s
//uint32_t wait_for_udp_packet_threshold = 0x000c0000; // 30.9s

uint8_t wait_for_udp_packet_retries = 0;

uint16_t signal_strength_retry_cnt = 0;
//uint16_t signal_strength_retry_threshold_init = 0x00000100; // 13.5s
uint16_t signal_strength_retry_threshold_init = 0x00000800; // 110.5s
uint16_t signal_strength_retry_threshold_socket = 0x00000200; // 27.5s

uint16_t get_eeprom_ack_read_address = 0;

bool reset_after_deactivate_pdp_ctx = false;
bool bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx = false;
uint16_t bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx_firmware_checksum = 0;

/* Info from ewmarcomm@embeddedworks.net email (yahoo) on 11/21/2017
   APN = m2mglobal

   Device ID and Mobile # (MTN): 
   8901260882222862127F
   11/21/2018
*/

// AT+CMEE=2   // Enables the cellular module to report verbose error result codes
// AT+CREG?    // Verify the network registration
// - +CREG: 0,3 (no sim card, no antenna)
// - +CREG: 0,0 (sim card, antenna)
// AT+COPS=0   // Register the phone on the network. The cellular module automatically registers
// itself on the cellular network. This command is necessary only if the automatic
// registration failed (AT+CREG? returns 0,0)
// AT+COPS?    // Read operator name
// - not neccessary, just returns '+COPS: 0'

// AT+CIND? // signal, service, gprs, sim detected
// - +CIND: 0,0,0,0,0,0,0,1,0,0,0,0 (no sim card, no antenna)
// - +CIND: 0,0,0,0,0,0,0,0,0,0,0,0 (sim card, no antenna)
// - +CIND: 0,2,0,0,0,0,0,0,1,0,0,0 (sim card, antenna - both mine and the taoglas return 2)

// AT+UANTR=0 // Antenna detection
// - +UANTR: 0,-1 (no antenna)
// - +UANTR: 0,-1 (antenna)

// AT+UPSD=0,1,"m2mglobal"  // Configure PDP-context parameters
// AT+UPSD=0                // Check the configuration
// AT+UPSDA=0,1             // Store configuration in non-volatile memory (NVM)
// AT+UPSDA=0,3             // Activate PDP-context
// AT+UPSND=0,0             // Check IP addresses assigned
// AT+UPSND=0,1             // Check DNS assigned

// AT+USOCR=17                              // Create a UDP socket
// AT+UDNSRN=0,"echo.u-blox.com"            // DNS resolution of the URL
// AT+USOST=0,"195.34.89.241",13,5,"Hello"  // Write 5 characters to the server
// AT+USORF=0,27                            // Read 27 characters

// http://www.avrfreaks.net/forum/array-strings-flash-1
// - use __flash instead of PROGMEM
// - These no longer need pgm_read*() dereference

typedef struct {
    const __flash char *cmd;
    const __flash char *expect_line1;
    const __flash char *expect_line2;
    const __flash char *expect_line3;
    const __flash char *expect_line4;
    uint8_t delay;
} SaraCmd;

// I want to first reset the SARA since the mcu may just have been re-programmed, but the SARA may have state left over.
// Performs a MT silent reset (with detach from network and saving of NVM parameters) with reset of the SIM card, then to full functionality
static const __flash char cmd_reset_cmd[] = "AT+CFUN=15\r";
static const __flash char cmd_reset_expect_line1[] = "OK";

// UART data rate configuration
//static const __flash char cmd_uart_data_rate_cmd[] = "AT+IPR=38400\r";
static const __flash char cmd_uart_data_rate_cmd[] = "AT+IPR=9600\r";
//static const __flash char cmd_uart_data_rate_expect_line1[] = "OK";

// IMEI identification - Verizon needs to know this when registering the SIM card
// \r\n352753092858743\r\n\r\nOK\r\n
//     35-275309-285874-3
//static const __flash char cmd_imei_cmd[] = "AT+CGSN\r";

// Verbose errors
/*
// tmp
static const __flash char cmd_verbose_errors_cmd[] = "AT+CMEE=2\r";
static const __flash char cmd_verbose_errors_expect_line1[] = "OK";
// end tmp
*/

// AT+CGDCONT=1,"IP","CHAP:internet.t-d1.de"
// AT+CGDCONT=1,"IP","APN_name","1.2.3.4",0,0
// AT+CGDCONT=1,"IP","hologram"\r (29)
// AT+CGDCONT=1,"IP","VZWINTERNET"\r (32)
// AT+UPSD=0,1,"hologram" (22)

/* After skipping activate_pdp_ctx
AT+CSQ\r
AT+CSQ\r\r\n+CSQ: 14,99\r\n\r\nOK\r\n

AT+CGDCONT=1,"IP","hologram"\r
AT+CGDCONT=1,"IP","hologram"\r\r\nOK\r\n

AT+USOCR=17\r
AT+USOCR=17\r\r+USOCR: 0\r\n\r\nOK\r\n

AT+USOST=0,\"34.233.140.210\",6133,7\r
AT+USOST=0,\"34.233.140.210\",6133,7\r\r\n@

'1' '0' '25' '0' \n '0' '14'
'1' '0' '25' '0' \n '0' '14'   \r\n+USOST: 0,7\r\n\r\nOK\r\n   \r\n+UUSORF: 0,20\r\n
*/

// Configure PDP-context parameters
//static const __flash char cmd_set_apn_cmd[] = "AT+UPSD=0,1,\"m2mglobal\"\r"; // T-Mobile
//static const __flash char cmd_set_apn_cmd[] = "AT+UPSD=0,1,\"hologram\"\r"; // Hologram
static const __flash char cmd_set_apn_cmd[] = ""; // formatted dynamically
static const __flash char cmd_set_apn_expect_line1[] = "OK";

// Store configuration in non-volatile memory (NVM)
//static const __flash char cmd_store_in_non_volatile_cmd[] = "AT+UPSDA=0,1\r";
//static const __flash char cmd_store_in_non_volatile_expect_line1[] = "OK";

// Load configuration from non-volatile memory (NVM)
//static const __flash char cmd_load_from_non_volatile_cmd[] = "AT+UPSDA=0,2\r";
//static const __flash char cmd_load_from_non_volatile_expect_line1[] = "OK";

// Command echo E
static const __flash char cmd_echo_off_cmd[] = "ATE0\r";
static const __flash char cmd_echo_off_expect_line1[] = "OK";

// Signal quality
// \r\n+CESQ: 99,99,255,255,18,24\r\n\r\nOK\r\n
// \r\n+CESQ: 99,99,255,255,16,22\r\n\r\nOK\r\n
static const __flash char cmd_signal_qual_cmd[] = "AT+CESQ\r";
static const __flash char cmd_signal_qual_expect_line1[] = "";
static const __flash char cmd_signal_qual_expect_line2[] = "";
static const __flash char cmd_signal_qual_expect_line3[] = "OK";

// Power off
static const __flash char cmd_power_off_cmd[] = "AT+CPWROFF\r";
static const __flash char cmd_power_off_expect_line1[] = "OK";

//static const SaraCmd cmds[] PROGMEM = {
//static const SaraCmd cmds[] = {
static const __flash SaraCmd const __flash cmds_init[] = {
    {
        cmd_uart_data_rate_cmd,
        NULL,       // NULL means we are not expecting a line
        NULL,
        NULL,
        NULL,
        2
    },
    /*
    {
        cmd_imei_cmd,
        NULL,       // NULL means we are not expecting a line
        NULL,
        NULL,
        NULL,
        2
    },
    */
    {
        cmd_reset_cmd,
        cmd_reset_expect_line1,
        NULL,
        NULL,
        NULL,
        60
    },
    {
        cmd_uart_data_rate_cmd,
        NULL,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_echo_off_cmd,
        cmd_echo_off_expect_line1,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_set_apn_cmd,
        cmd_set_apn_expect_line1,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_signal_qual_cmd,
        cmd_signal_qual_expect_line1,
        cmd_signal_qual_expect_line2,
        cmd_signal_qual_expect_line3,
        NULL,
        0  // ignored, timer1 is not being used for cmds_init - cmd_signal_qual_cmd
    }
};

// Activate PDP-context
//static const __flash char cmd_activate_pdp_ctx_cmd[] = "AT+UPSDA=0,3\r";
static const __flash char cmd_activate_pdp_ctx_cmd[] = "AT+CGACT=1,1\r";
static const __flash char cmd_activate_pdp_ctx_expect_line1[] = "OK";
// +CME ERROR: Unspecified GPRS error

// don't need to do this, since hologram's web UI will tell us
// PS (Packet Switched) operator selection
//static const __flash char cmd_get_network_cmd[] = "AT+UCGOPS?\r"; // +UCGOPS: 0,0,"T-Mobile"\r\n\r\nOK\r\n
//static const __flash char cmd_get_network_expect_line1[] = "";
//static const __flash char cmd_get_network_expect_line2[] = "";
//static const __flash char cmd_get_network_expect_line3[] = "OK";

// Get the dynamic IP address assigned during PDP context activation
// I don't need this, but I am curious if this is what the server receives (ie are there any NATs)
// +UPSND: 0,0,"10.30.204.4"\r\n
/*
  static const __flash char cmd10_cmd[] = "AT+UPSND=0,0\r";
  static const __flash char cmd10_expect_line1[] = "";
  static const __flash char cmd10_expect_line2[] = "";
  static const __flash char cmd10_expect_line3[] = "OK";
*/

// Create a UDP socket
static const __flash char cmd_create_udp_socket_cmd[] = "AT+USOCR=17\r"; // UDP
//static const __flash char cmd_create_udp_socket_cmd[] = "AT+USOCR=6\r"; // TCP
static const __flash char cmd_create_udp_socket_expect_line1[] = "+USOCR: 0";
static const __flash char cmd_create_udp_socket_expect_line2[] = "";
static const __flash char cmd_create_udp_socket_expect_line3[] = "OK";

// DNS resolution of the URL
// +UDNSRN: "195.34.89.241"\r\n\r\nOK\r\n
/*
  static const __flash char cmd8_cmd[] = "AT+UDNSRN=0,\"echo.u-blox.com\"\r";
  static const __flash char cmd8_expect_line1[] = "";
  static const __flash char cmd8_expect_line2[] = "";
  static const __flash char cmd8_expect_line3[] = "OK";
*/

// Write 5 characters to the server
// read_buf must be 43 + length of packet, to hold this echo (we could turn off echo)
/*
  static const __flash char cmd_sendto_cmd[] = "AT+USOST=0,\"195.34.89.241\",13,5,\"Hellk\"\r";
  static const __flash char cmd_sendto_expect_line1[] = ""; // +USOST: 0,5
  static const __flash char cmd_sendto_expect_line2[] = "";
  static const __flash char cmd_sendto_expect_line3[] = "OK";
*/

// This seems to force dynamic ip to reset (ie if you set it then, comment it out the next time)
//static const __flash char cmd_set_ip_cmd[] = "AT+UPSD=0,7,\"10.30.204.4\"\r";
//static const __flash char cmd_set_ip_expect_line1[] = "OK";

static const __flash SaraCmd const __flash cmds_create_udp_socket[] = {
    /*
    // tmp
    {
        cmd_verbose_errors_cmd,
        cmd_verbose_errors_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    // end tmp
    */
    /*
    // tmp
    {
        cmd_set_ip_cmd,
        cmd_set_ip_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    // end tmp
    */
    {
        cmd_uart_data_rate_cmd,
        NULL,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_echo_off_cmd,
        cmd_echo_off_expect_line1,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_set_apn_cmd,
        cmd_set_apn_expect_line1,
        NULL,
        NULL,
        NULL,
        //200 // TODO: try to lower this
        2
    },
    {
        cmd_create_udp_socket_cmd,
        cmd_create_udp_socket_expect_line1,
        cmd_create_udp_socket_expect_line2,
        cmd_create_udp_socket_expect_line3,
        NULL,
        8
    }
};

static const __flash SaraCmd const __flash cmds_create_udp_socket_signal_qual[] = {
    {
        cmd_uart_data_rate_cmd,
        NULL,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_echo_off_cmd,
        cmd_echo_off_expect_line1,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_set_apn_cmd,
        cmd_set_apn_expect_line1,
        NULL,
        NULL,
        NULL,
        2
    },
    {
        cmd_signal_qual_cmd,
        cmd_signal_qual_expect_line1,
        cmd_signal_qual_expect_line2,
        cmd_signal_qual_expect_line3,
        NULL,
        8
    },
    /*
    {
        cmd_activate_pdp_ctx_cmd,
        cmd_activate_pdp_ctx_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    */
    /*
    {
        cmd_get_network_cmd,
        cmd_get_network_expect_line1,
        cmd_get_network_expect_line2,
        cmd_get_network_expect_line3,
        NULL,
        1
    },
    */
    {
        cmd_create_udp_socket_cmd,
        cmd_create_udp_socket_expect_line1,
        cmd_create_udp_socket_expect_line2,
        cmd_create_udp_socket_expect_line3,
        NULL,
        8
    }
};

//static const __flash char cmd_sendto_cmd[] = "AT+USOST=0,\"34.233.140.210\",6133,5\r";
static const __flash char cmd_sendto_cmd[] = ""; // formatted dynamically
static const __flash char cmd_sendto_expect_line1[] = "@";

static const __flash char cmd_sendto_data_cmd[] = ""; // formatted dynamically
static const __flash char cmd_sendto_data_expect_line1[] = ""; // +USOST: 0,5
static const __flash char cmd_sendto_data_expect_line2[] = "";
static const __flash char cmd_sendto_data_expect_line3[] = "OK";

// tmp
//static const __flash char cmd_creg_cmd[] = "AT+CREG?\r"; // +CREG: 0,1 (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+CPIN?\r"; // +CPIN: READY (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+COPS?\r"; // +COPS: 0,0,"T-Mobile" (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+CIND?\r"; // +CIND: 0,1,1,0,0,0,0,0,2,0,0,0 (good)
                                                           // +CIND: 0,2,1,0,0,0,0,0,2,0,0,0 (bad) (only difference is signal quality)
//static const __flash char cmd_creg_cmd[] = "AT+UREG?\r"; // +CME ERROR: unknown (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+UGCNTRD\r"; // +UGCNTRD: 4,0,0,0,0 (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+UDCONF=66\r"; // +CME ERROR: operation not supported (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+UPSND=0,0\r"; // +UPSND: 0,0,"10.30.204.4" (good)
                                                               // +UPSND: 0,0,"72.250.82.68" (good2) (what server sees)
                                                               // +UPSND: 0,0,"72.250.83.55" (good3) (what server sees)
                                                               // +UPSND: 0,0,"21.102.12.22" (bad)
                                                               // +UPSND: 0,0,"30.82.238.190" (bad again)
                                                               // +UPSND: 0,0,"33.55.197.14" (bad again)
                                                               // +UPSND: 0,0,"33.52.137.210" (bad2)
//static const __flash char cmd_creg_cmd[] = "AT+CGPADDR=\r";  // +CGPADDR: 4,"10.30.204.4" (good)
                                                               // +CGPADDR: 4,"72.250.82.68" (good2) (what server sees)
                                                               // +CGPADDR: 4,"72.250.83.55" (good3) (what server sees)
                                                               // +CGPADDR: 4,"30.72.116.222" (bad)
                                                               // +CGPADDR: 4,"21.102.12.22" (bad again)
                                                               // +CGPADDR: 4,"33.55.197.14" (bad again)
                                                               // +CGPADDR: 4,"30.74.232.220" (bad2)

// Why would the Packet switched network-assigned data (+UPSND) be different than the PDP address +CGPADDR

//static const __flash char cmd_creg_cmd[] = "AT+UPSD=0\r";
/* (bad SIM card and good SIM card)
  +UPSD: 0,0,0
  +UPSD: 0,1,"m2mglobal"
  +UPSD: 0,2,""
  +UPSD: 0,4,"0.0.0.0"
  +UPSD: 0,5,"0.0.0.0"
  +UPSD: 0,6,0
  +UPSD: 0,7,"0.0.0.0"
  +UPSD: 0,8,0
  ...
  +UPSD: 0,19,0
*/

//static const __flash char cmd_creg_cmd[] = "AT+CGDCONT?\r"; // OK (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+GCAP\r"; // +GCAP: +FCLASS,+CGSM (bad SIM card and good SIM card)
//static const __flash char cmd_creg_cmd[] = "AT+UPCO=2,4\r"; // +

//static const __flash char cmd_creg_expect_line1[] = "";
// end tmp

static const __flash SaraCmd const __flash cmds_send_udp_packet[] = {
    /*
    // tmp
    {
        cmd_creg_cmd,
        cmd_creg_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    // end tmp
    */
    {
        cmd_sendto_cmd,
        cmd_sendto_expect_line1,
        NULL,
        NULL,
        NULL,
        8
    },
    {
        cmd_sendto_data_cmd,
        cmd_sendto_data_expect_line1,
        cmd_sendto_data_expect_line2,
        cmd_sendto_data_expect_line3,
        NULL,
        0
    }
};

// Read 27 characters
// +USORF: 0,"195.34.89.241",13,26,"16 FEB 2018 00:04:33 CET\r\n"\r\n\r\nOK\r\n
//static const __flash char cmd_receive_from_cmd[] = "AT+USORF=0,22\r";
static const __flash char cmd_receive_from_cmd[] = ""; // formatted dynamically
static const __flash char cmd_receive_from_expect_line1[] = "";
static const __flash char cmd_receive_from_expect_line2[] = "";
static const __flash char cmd_receive_from_expect_line3[] = "OK";
//static const __flash char cmd_receive_from_expect_line4[] = "OK";

static const __flash SaraCmd const __flash cmds_receive_from[] = {
    {
        cmd_receive_from_cmd,
        cmd_receive_from_expect_line1,
        cmd_receive_from_expect_line2,
        cmd_receive_from_expect_line3,
        NULL,
        1
    }
};

// Deactivate PDP-context
//static const __flash char cmd_deactivate_pdp_ctx_cmd[] = "AT+UPSDA=0,4\r";
static const __flash char cmd_deactivate_pdp_ctx_cmd[] = "AT+CGACT=0,1\r";
static const __flash char cmd_deactivate_pdp_ctx_expect_line1[] = ""; // could be OK or ERROR, both are fine

static const __flash SaraCmd const __flash cmds_deactivate_pdp_ctx[] = {
    /*
    {
        cmd_deactivate_pdp_ctx_cmd,
        cmd_deactivate_pdp_ctx_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    */
    {
        cmd_power_off_cmd,
        cmd_power_off_expect_line1,
        NULL,
        NULL,
        NULL,
        10
    }
};

/*
static const __flash char cmd_ugind_cmd[] = "AT+UGIND=1\r";
static const __flash char cmd_ugind_expect_line1[] = "OK";

// Configure cellular location sensor (CellLocate)
static const __flash char cmd_uloccell_cmd[] = "AT+ULOCCELL=1\r";
static const __flash char cmd_uloccell_expect_line1[] = "OK";

// Ask for localization information
// TODO: response_type=1, timeout=60, accuracy=5000
// AT+ULOCCELL=0:
// accuracy=5000 -> 1214
// accuracy=1000 -> 1214
// accuracy=1 -> 1214

// AT+ULOCCELL=1:
// accuracy=1 -> 
static const __flash char cmd_uloc_cmd[] = "AT+ULOC=2,2,1,60,1\r";
static const __flash char cmd_uloc_expect_line1[] = "OK";

static const __flash SaraCmd const __flash cmds_uloccell[] = {
    {
        cmd_set_apn_cmd,
        cmd_set_apn_expect_line1,
        NULL,
        NULL,
        NULL,
        200 // TODO: try to lower this
    },
    {
        cmd_activate_pdp_ctx_cmd,
        cmd_activate_pdp_ctx_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    {
        cmd_ugind_cmd,
        cmd_ugind_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    {
        cmd_uloccell_cmd,
        cmd_uloccell_expect_line1,
        NULL,
        NULL,
        NULL,
        1
    },
    {
        cmd_uloc_cmd,
        cmd_uloc_expect_line1,
        NULL,
        NULL,
        NULL,
        250 // TODO: try to lower this
    },
    //{
        //cmd_deactivate_pdp_ctx_cmd,
        //cmd_deactivate_pdp_ctx_expect_line1,
        //NULL,
        //NULL,
        //NULL,
        //1
    //},
};
*/

//const __flash SaraCmd * const __flash cmds = cmds_init; // fails to compile when being re-assigned
const __flash SaraCmd const __flash * cmds = cmds_init; // compiles
//cmds = cmds_init; // compiles
//SaraCmd const __flash * cmds = cmds_init; // compiles, but seems less restrictive than it should be

uint8_t cmds_index = 0;
//const __flash SaraCmd *cmd = &(cmds[cmds_index]);
const __flash SaraCmd *cmd;
//const __flash char *cmd_p = cmd->cmd;
const __flash char *cmd_p;


char *itoa_decimal(uint8_t x) {
    static char ret[4];

    uint8_t x1 = x / 100;
    x -= x1 * 100;
    uint8_t x2 = x / 10;
    x -= x2 * 10;
    uint8_t x3 = x;

    uint8_t pos = 0;
    if (x1 > 0) {
        uint8_t y1 = x1 + '0';
        ret[pos++] = y1;
    }

    if (x1 > 0 || x2 > 0) {
        uint8_t y2 = x2 + '0';
        ret[pos++] = y2;
    }

    uint8_t y3 = x3 + '0';
    ret[pos++] = y3;
    ret[pos] = '\0';

    return ret;
}


void enable_pullups_on_unused_pins(void) {
    /*
      If some pins are unused, it is recommended to ensure that these pins have a defined level. Even though most of
      the digital inputs are disabled in the deep sleep modes as described above, floating inputs should be avoided to
      reduce current consumption in all other modes where the digital inputs are enabled (Reset, Active mode and Idle
      mode). The simplest method to ensure a defined level of an unused pin, is to enable the internal pull-up.
    */

    // No unused pins!
}


void led_setup(void) {
    PORTA |= (1 << PA0); // LED off
    DDRA |= (1 << PA0); // Configure LED as output
}


void led_on(void) {
    PORTA &= ~(1 << PA0); // LED on
}


void led_off(void) {
    PORTA |= (1 << PA0); // LED off
}


void led_bit_bang_data(uint8_t data) {
    // send bits 7..0
    for (uint8_t i = 0; i < 8; i++) {
        // consider leftmost bit
        // set line high if bit is 1, low if bit is 0
        if (data & 0x80) {
            //output_high(SD_DI);
            led_off();
        } else {
            //output_low(SD_DI);
            led_on();
        }

        _delay_ms(1);

        // shift byte left so next bit will be leftmost
        data <<= 1;
    }
}


bool power_down_after_deactivate_pdp_ctx = false;
const uint16_t power_down_after_deactivate_pdp_ctx_cnt_base_address = 2;
uint8_t power_down_after_deactivate_pdp_ctx_cnt = 0;


void clear_power_down_after_deactivate_pdp_ctx_cnt(void) {
    if (power_down_after_deactivate_pdp_ctx_cnt != 1) {
        power_down_after_deactivate_pdp_ctx_cnt = 1;
        eeprom_write_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address, 1);
    }
}


void read_power_down_after_deactivate_pdp_ctx_cnt(bool we_were_reset_by_wdt) {
    // Read power_down_after_deactivate_pdp_ctx_cnt from eeprom
    power_down_after_deactivate_pdp_ctx_cnt = eeprom_read_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address);
    if (power_down_after_deactivate_pdp_ctx_cnt == 0 || power_down_after_deactivate_pdp_ctx_cnt == 0xff)
        power_down_after_deactivate_pdp_ctx_cnt = 1;

    if (!we_were_reset_by_wdt) {
        // Since we were not reset by WDT, we must have been powered up manually by on/off switch.
        // Therefore we should clear the cnt (set back to 1)
        clear_power_down_after_deactivate_pdp_ctx_cnt();
    }
}


void set_power_down_after_deactivate_pdp_ctx(void) {
    power_down_after_deactivate_pdp_ctx = true;

    // 2min (1), 4min (2), 16min (8), 1hr (30), 4hr (120)
    if (power_down_after_deactivate_pdp_ctx_cnt == 1)
        eeprom_write_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address, 2);
    else if (power_down_after_deactivate_pdp_ctx_cnt == 2)
        eeprom_write_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address, 8);
    else if (power_down_after_deactivate_pdp_ctx_cnt == 8)
        eeprom_write_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address, 30);
    else if (power_down_after_deactivate_pdp_ctx_cnt == 30)
        eeprom_write_byte((void *)power_down_after_deactivate_pdp_ctx_cnt_base_address, 120);
}


// ErrorCode impl is currently taking up 10% of 4k program memory usage (and year/month/day are still not implemented)
// I could possibly simplify error_codes_setup by storing the index in the eeprom
enum {
    ERROR_CODE_BATT_LOW, // 0x00
    ERROR_CODE_USART_FRAME_ERROR, // 0x01
    ERROR_CODE_USART_DATA_OVERRUN, // 0x02
    ERROR_CODE_USART_PARITY_ERROR, // 0x03
    ERROR_CODE_UNEXPECTED_NON_USORF_CMD, // 0x04
    ERROR_CODE_UNEXPECTED_LINE, // 0x05
    ERROR_CODE_STATE_READ_AWAITING_CMD, // 0x06
    ERROR_CODE_STATE_READ_GOT_CMD1, // 0x07
    ERROR_CODE_STATE_READ_GOT_CMD2, // 0x08
    ERROR_CODE_STATE_READ_GOT_EXPECT_LINE1, // 0x09
    ERROR_CODE_STATE_READ_GOT_EXPECT_LINE2, // 0x0a
    ERROR_CODE_STATE_READ_GOT_EXPECT_LINE3, // 0x0b
    ERROR_CODE_READ_BUF_OVERRUN, // 0x0c
    ERROR_CODE_BATT_LOW_0, // 0x0d
    ERROR_CODE_TEMP_SENSOR_CONFIGURE_FAILED, // 0x0e
    ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED, // 0x0f
    ERROR_CODE_EXTERNAL_EEPROM_WRITE_FAILED, // 0x10
    ERROR_CODE_EXTERNAL_EEPROM_VERIFY_FAILED, // 0x11
    ERROR_CODE_EXTERNAL_EEPROM_CHECKSUM_FAILED, // 0x12
    ERROR_CODE_UNEXPECTED_UUSORD, // 0x13
    ERROR_CODE_UNEXPECTED_UUPSDD, // 0x14
    ERROR_CODE_UNEXPECTED_UUSOCL, // 0x15
    ERROR_CODE_WAIT_FOR_UDP_PACKET_RETRIES, // 0x16
    ERROR_CODE_UNEXPECTED_NON_UPSD_CMD, // 0x17
    ERROR_CODE_TIMER1_RUNNING1, // 0x18
    ERROR_CODE_TIMER1_RUNNING2, // 0x19
};


typedef struct {
    uint16_t days;
    uint16_t time_of_day;
    uint8_t code;
} ErrorCode;


uint8_t error_code_index = 0;
const uint8_t error_code_base_address = 32;
const uint8_t error_code_num_of_entries = 44; // (256 - 32) / 5


void error_codes_setup(void) {
    // Set error_code_index to oldest entry
    error_code_index = 0;

    ErrorCode ec;
    ErrorCode ec_oldest = { 0xffff, 0xffff, 0xff };

    for (uint8_t i = 0; i < error_code_num_of_entries; ++i) {
        eeprom_read_block(&ec, (void *)(error_code_base_address + i * sizeof(ec)), sizeof(ec));
        if (ec.days == 0xffff && ec.time_of_day == 0xffff) {
            // Found an unused entry, use it
            error_code_index = i;
            break;
        }

        if (ec.days == 0xffff)
            ec.days = 0;
        if (ec.time_of_day == 0xffff)
            ec.time_of_day = 0;

        bool is_older = false;
        if (ec.days < ec_oldest.days)
            is_older = true;

        if (ec.days == ec_oldest.days && ec.time_of_day < ec_oldest.time_of_day)
            is_older = true;

        if (is_older) {
            ec_oldest = ec;
            error_code_index = i;
        }
    }
}


void error_codes_store_code(uint8_t code) {
    ErrorCode ec;
    ec.days = fathom_days_offset;
    ec.time_of_day = fathom_time_of_day;
    ec.code = code;

    eeprom_write_block(&ec, (void *)(error_code_base_address + error_code_index * sizeof(ec)), sizeof(ec));

    if (error_code_index < error_code_num_of_entries - 1)
        ++error_code_index;
    else
        error_code_index = 0;
}


void usart_enable(void) {
    PORTB |= (1 << PB2); // RTS up
    DDRB |= (1 << PB2); // Configure RTS as output

    PORTB |= (1 << PB0); // TXD up
    DDRB |= (1 << PB0); // Configure TXD as output

    // Set baud rate
    //uint16_t baud = 25; // 2400
    //uint16_t baud = 12; // 4800
    //uint16_t baud = 6; // 9600 (measured to be 8915)
    //uint16_t baud = 5; // 9600 (measured to be 10400)
    //uint16_t baud = 12; // 9600 (at U2X0 = 1)
    //uint16_t baud = 0; // 115200 (at U2X0 = 1) (measured to be 124906)
    //uint16_t baud = 1; // 115200 (at U2X0 = 1) (measured to be 62200)
    //uint16_t baud = 8; // 115200 (at U2X0 = 1 and 8MHz) (measured to be 111086)
    uint16_t baud = 0; // 115200 (at U2X0 = 0 and 1.8432MHz) (measured to be 115200)
    UBRR0H = (uint8_t)(baud >> 8);
    UBRR0L = (uint8_t)baud;

    //UCSR0A |= (1 << U2X0); // Set USART to Asynchronous Double Speed mode

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}


void usart_disable(void) {
    UCSR0B = 0x00;

    DDRB &= ~(1 << PB2); // Configure RTS as input
    PORTB &= ~(1 << PB2); // RTS down

    // TXD draws 18mA(!) while enabled and configured as output
    DDRB &= ~(1 << PB0); // Configure TXD as input
    PORTB &= ~(1 << PB0); // TXD down
}


uint8_t usart_read(bool *frame_error, bool *data_overrun, bool *parity_error) {
    // Wait for data to be received
    //while ( !(UCSRA & (1 << RXC)) );

    uint8_t flags = UCSR0A;

    *frame_error = flags & (1 << FE0);
    *data_overrun = flags & (1 << DOR0);
    *parity_error = flags & (1 << UPE0);

    // Get and return received data from buffer
    return UDR0;
}


void usart_write(uint8_t data) {
    // Put data into buffer, sends the data
    UDR0 = data;
}


void usart_write_wait(uint8_t data) {
    // Wait for empty transmit buffer
    while ( !(UCSR0A & (1 << UDRE0)) );

    // Put data into buffer, sends the data
    UDR0 = data;
}


void usart_read_any(void) {
    bool frame_error;
    bool data_overrun;
    bool parity_error;

    while (UCSR0A & (1 << RXC0)) {
        usart_read(&frame_error, &data_overrun, &parity_error);
        if (frame_error) {
            usart_write('f');
            usart_write('e');
            for (;;) {
            }
        } else if (data_overrun) {
            usart_write('d');
            usart_write('o');
            for (;;) {
            }
        } else if (parity_error) {
            usart_write('p');
            usart_write('e');
            for (;;) {
            }
        } else {
        }
    }
}


//#define TIMER0_200MS 0x00 // 264ms
//#define TIMER0_200MS 0x3e // 199.5ms
//#define TIMER0_200MS 0x40 // 197ms
//#define TIMER0_200MS 0x80 // 132ms
//#define TIMER0_200MS 0xc0 // 65ms

//#define TIMER0_200MS 0x9e // 54.4ms
//#define TIMER0_200MS 0x3e // 107.8ms
//#define TIMER0_200MS 0x20 // 124.4ms
#define TIMER0_125MS 0x1f // 125ms
//#define TIMER0_200MS 0x00 // 142.5ms

//#define TIMER0_250MS 0x0d // 0.2493s (250ms - 700us)
//#define TIMER0_250MS 0x0c // 0.2503s (250ms + 300us)

//#define TIMER0_250MS 0x0e // 59.90 / 30 / 8 = 249.6ms
//#define TIMER0_250MS 0x0d // 60.14 / 30 / 8 = 250.6ms
//#define TIMER0_250MS 0x0c // 60.39 / 30 / 8 = 251.6ms
//#define TIMER0_250MS 0x05 // 62.12 / 30 / 8 = 258.8ms

volatile uint8_t timer0_blink_cnt = 0;
volatile uint8_t timer0_led_state;


void timer0_setup(void) {
    TIMSK |= (1 << TOIE0); // Timer/Counter1, Overflow Interrupt Enable
}


void timer0_start(void) {
    timer0_led_state = 0;
    TCNT0 = TIMER0_125MS; // load the timer counter value
    TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); // Clock Select (starts the timer)
}


volatile bool timer0_done_blinking = false;
uint8_t time_keeper_intermediate_count = 0;


void timer0_start_keeping_time(void) {
    timer0_done_blinking = true;

    TCNT0 = TIMER0_125MS; // load the timer counter value
    TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); // Clock Select (starts the timer)
}


void timer0_stop(void) {
    TCCR0B = (0 << CS02) | (0 << CS01) | (0 << CS00); // Clock Select (stops the timer)
}


//volatile bool send_if_needed_set_current_temp_timer_fired = false;
volatile bool send_if_needed_set_daily_timer_fired = false;
volatile bool send_if_needed_heartbeat_timer_fired = true; // set to true so heartbeat will be send immediately upon startup


// Timer0 Overflow interrupt
ISR(TIMER0_OVF_vect) {
    TCNT0 = TIMER0_125MS; // load the timer counter value

    if (timer0_done_blinking) {
        // Keep time
        if (++time_keeper_intermediate_count == 16) {
            time_keeper_intermediate_count = 0;

            // It's been 2s
            ++fathom_time_of_day;

            /*
            // TIMER0_250MS does not give us exactly 2s, so we need to adjust every so often
            //if (fathom_time_of_day % 2 == 0) {
            if (fathom_time_of_day % 4 == 0) {
                // #14 (at TIMER0_250MS = 0x0e) (at % 2):
                //  0  -> 59.89s
                //  no -> 59.90s
                //  -1 -> 59.92s
                //  -3 -> 59.94s
                //  -6 -> 59.99s 599.5s
                //  -7 -> 60.01s 600.0s 59m59s
                //  -8 -> 60.02s (2 or 3 seconds fast every 3 hrs)
                //     if -9 turns out to be slow, the answer would be something like -8.5, which means we should go to % 4 and -17
                //     because every 4s we subtract 8.5, which is the same as saying every 8s we subtract 17
                //  -9 -> (1s or 2s fast every 1hr, 10s fast every 12hr)
                //     if -10 turns out to be slow, the answer would be something like -9.5, which means we should go to % 4 and -19
                //     because every 4s we subtract 9.5, which is the same as saying every 8s we subtract 19
                // -10 -> (0s or 1s slow every 1hr, 4s slow every 3hr, xs slow every 12hr)

                // #14 (at TIMER0_250MS = 0x0e) (at % 4):
                //   0 -> 9s fast every 1hr
                //  -2 -> 7s fast every 1hr
                //  -8 -> 4s fast every 1hr
                // -10 -> 3s fast every 1hr
                // -12 -> 3s fast every 1hr
                // -14 -> 2s fast every 1hr
                // -19 -> 2min fast every 1hr
                // -30 -> 113s fast every 1hr

                // #14 (at TIMER0_250MS = 0x0d) (at % 4):
                //   0 -> 7s slow every 1hr
                //  10 -> 2s slow every 1hr
                //  11 -> 2s slow every 1hr
                //  12 -> 1s slow every 1hr
                //  13 -> 1s slow every 1hr
                //  14 -> s slow every 1hr, 1s fast every 3hr
                //  15 -> 1s fast every 1hr, 3s fast every 3hr
                TCNT0 = TIMER0_125MS + 14; // load the timer counter value
            }
            */

            /*
            // tmp
            //if (fathom_time_of_day % 300 == 0) {
            if (fathom_time_of_day % 30 == 0) {
                led_on();
                led_off();
            }
            // end tmp
            */

            if (fathom_time_of_day > 43200) { // 60 * 60 * 24 / 2
                fathom_time_of_day = 0;
                ++fathom_days_offset;
                set_daily_temp_last_sent = 0;
                //midnight_has_passed = true;
            }

            batt_low_on_timer();
            water_low_on_timer();
            temp_low_on_timer();

            /*
            if (++power_down_cnt_set_current_temp == set_current_temp_interval) {
                power_down_cnt_set_current_temp = 0;
                send_if_needed_set_current_temp_timer_fired = true;
            }
            */

            if (++power_down_cnt_set_daily_temp != 60 * 10 / 2) { // check temp_sensor every 10 min
                power_down_cnt_set_daily_temp = 0;
                send_if_needed_set_daily_timer_fired = true;
            }

            if (++power_down_cnt_heartbeat == heartbeat_interval) {
                power_down_cnt_heartbeat = 0;

                // Is it between 7am and 9am?
                const uint16_t seven_am = 12600; // 60 * 60 * 7 / 2
                const uint16_t nine_am = 16200; // 60 * 60 * 9 / 2
                //if (fathom_time_of_day > (seven_am + postpone_set_daily_by) && fathom_time_of_day < (nine_am + postpone_set_daily_by)) {
                if (fathom_time_of_day > seven_am && fathom_time_of_day < (nine_am + postpone_set_daily_by)) {
                    // Don't send HEARTBEAT because SET_DAILY is used as a heartbeat
                } else {
                    send_if_needed_heartbeat_timer_fired = true;
                }
            }
        }
    } else {
        // Blink
        if (timer0_blink_cnt == 0) {
            timer0_stop();
            led_off();

            state_write = STATE_WRITE_CMD_DELAY_OVER;
        } else {
            if (timer0_led_state >= 0 && timer0_led_state <= 3) {
                led_off();

                ++timer0_led_state;
            } else if (timer0_led_state >= 4 && timer0_led_state <= 7) {
                led_on();

                if (timer0_led_state == 7) {
                    timer0_led_state = 0;

                    --timer0_blink_cnt;
                } else {
                    ++timer0_led_state;
                }
            }
        }
    }
}


bool external_eeprom_set_reg(uint8_t reg_address, uint8_t val) {
    // ST and SAD+W
    if (pht_i2c_write_byte(true, false, 0xa2)) { // 1010 0010
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte1
    if (pht_i2c_write_byte(false, false, 0x00)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte2
    if (pht_i2c_write_byte(false, false, reg_address)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(2);

    // SR and SAD+R
    if (pht_i2c_write_byte(false, true, val)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(6);
    return true;
}


bool external_eeprom_page_write(uint8_t page_num, uint8_t *firmware) {
    uint8_t address_byte1 = page_num >> 2 & 0xff;
    uint8_t address_byte2 = page_num << 6 & 0xff;

    // ST and SAD+W
    if (pht_i2c_write_byte(true, false, 0xa2)) { // 1010 0010
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte1
    if (pht_i2c_write_byte(false, false, address_byte1)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte2
    if (pht_i2c_write_byte(false, false, address_byte2)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(2);

    for (uint8_t i = 0; i < 63; ++i) {
        if (pht_i2c_write_byte(false, false, firmware[i])) {
            return false; // Slave failed to ACK
        }
    }

    if (pht_i2c_write_byte(false, true, firmware[63])) {
        return false; // Slave failed to ACK
    }

    _delay_ms(6);
    return true;
}


bool external_eeprom_page_verify(uint8_t page_num, uint8_t *firmware) {
    uint8_t address_byte1 = page_num >> 2 & 0xff;
    uint8_t address_byte2 = page_num << 6 & 0xff;

    // ST and SAD+W
    if (pht_i2c_write_byte(true, false, 0xa2)) { // 1010 0010
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte1
    if (pht_i2c_write_byte(false, false, address_byte1)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte2
    if (pht_i2c_write_byte(false, false, address_byte2)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(2);

    // SR and SAD+R
    if (pht_i2c_write_byte(true, false, 0xa3)) { // 1010 0011
        return false; // Slave failed to ACK
    }

    _delay_ms(3);

    // DATA and NMAK and SP
    uint8_t val_read;
    for (uint8_t i = 0; i < 63; ++i) {
        val_read = pht_i2c_read_byte(false, false);
        if (val_read != firmware[i])
            return false;
    }

    val_read = pht_i2c_read_byte(true, true);
    if (val_read != firmware[63])
        return false;

    return true;
}


bool external_eeprom_page_checksum(uint8_t page_num, uint16_t *firmware_checksum) {
    uint8_t address_byte1 = page_num >> 2 & 0xff;
    uint8_t address_byte2 = page_num << 6 & 0xff;

    // ST and SAD+W
    if (pht_i2c_write_byte(true, false, 0xa2)) { // 1010 0010
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte1
    if (pht_i2c_write_byte(false, false, address_byte1)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // address byte2
    if (pht_i2c_write_byte(false, false, address_byte2)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(2);

    // SR and SAD+R
    if (pht_i2c_write_byte(true, false, 0xa3)) { // 1010 0011
        return false; // Slave failed to ACK
    }

    _delay_ms(3);

    // DATA and NMAK and SP
    uint8_t val_read;
    uint16_t sum = *firmware_checksum;
    for (uint8_t i = 0; i < 63; ++i) {
        val_read = pht_i2c_read_byte(false, false);
        sum = (uint16_t)(sum + val_read);
    }

    val_read = pht_i2c_read_byte(true, true);
    sum = (uint16_t)(sum + val_read);

    *firmware_checksum = sum;
    return true;
}


bool external_eeprom_checksum_verify(uint16_t firmware_checksum) {
    // Takes about 65s
    uint16_t firmware_checksum_local = 0;
    for (uint8_t i = 0; i <= firmware_page_num_last_received; ++i) {
        if (!external_eeprom_page_checksum(i, &firmware_checksum_local))
            return false;
    }

    if (firmware_checksum_local != firmware_checksum)
        return false;

    return true;
}


bool external_eeprom_get_reg(uint8_t reg_address, uint8_t *pval) {
    // ST and SAD+W
    if (pht_i2c_write_byte(true, false, 0xa2)) { // 1010 0010
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // SUB
    if (pht_i2c_write_byte(false, false, 0x00)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(1);

    // SUB
    if (pht_i2c_write_byte(false, false, reg_address)) {
        return false; // Slave failed to ACK
    }

    _delay_ms(2);

    // SR and SAD+R
    if (pht_i2c_write_byte(true, false, 0xa3)) { // 1010 0011
        return false; // Slave failed to ACK
    }

    _delay_ms(3);

    // DATA and NMAK and SP
    *pval = pht_i2c_read_byte(true, true);
    return true;
}


void external_eeprom_setup(void) {
    PORTC &= ~(1 << PC2); // write 0 to SCL
    DDRC &= ~(1 << PC2); // Configure SCL as input
    PORTA &= ~(1 << PA5); // write 0 to SDA
    DDRA &= ~(1 << PA5); // Configure SDA as input
}


void external_eeprom_test(void) {
    uint8_t reg_val_orig;
    uint8_t reg_val;

    bool ok = external_eeprom_get_reg(0x00, &reg_val_orig);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_set_reg(0x00, 0x60);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_get_reg(0x00, &reg_val);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    if (reg_val != 0x60) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_set_reg(0x00, 0x3d);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_get_reg(0x00, &reg_val);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    if (reg_val != 0x3d) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_set_reg(0x00, reg_val_orig);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    ok = external_eeprom_get_reg(0x00, &reg_val);
    if (!ok) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }

    if (reg_val != reg_val_orig) {
        error_codes_store_code(ERROR_CODE_EXTERNAL_EEPROM_TEST_FAILED);
        return;
    }
}


void pwr_on_pin_pull_low(void) {
    // Low level on PWR_ON pin for 150 ms min, but not for 1.5s
    DDRB |= (1 << PB3); // Configure PWR_ON as output
    _delay_ms(300);
    DDRB &= ~(1 << PB3); // Configure PWR_ON as input
}


void pwr_on_pin_pull_low_long(void) {
    // Low level on PWR_ON pin for 1.5s min
    DDRB |= (1 << PB3); // Configure PWR_ON as output
    _delay_ms(2000);
    DDRB &= ~(1 << PB3); // Configure PWR_ON as input
}


bool wake_up_sara(uint8_t state) {
    usart_enable();

    // Wake up SARA
    pwr_on_pin_pull_low();

    // delay for 5s
    timer1_delay_100ms_cnt = 50;
    timer1_start();
    state_write = state;
    return true;
}


/*
bool send_if_needed_set_current_temp(void) {
    if (!send_if_needed_set_current_temp_timer_fired)
        return false;

    send_if_needed_set_current_temp_timer_fired = false;

    //temp_sensor_get_temp(&set_current_temp_temperature_high, &set_current_temp_temperature_low);
    uint16_t current_temp = temp_sensor_get_temp();
    set_current_temp_temperature_high = HI_BYTE(current_temp);
    set_current_temp_temperature_low = LO_BYTE(current_temp);

    return wake_up_sara(STATE_WRITE_SET_CURRENT_TEMP_DELAY);
}
*/


// Fathom sends today's high, and today's low at 7am every morning (but only if a full day has elapsed)
bool send_if_needed_set_daily(void) {
    if (!send_if_needed_set_daily_timer_fired)
        return false;

    send_if_needed_set_daily_timer_fired = false;

    //uint8_t temperature_high;
    //uint8_t temperature_low;
    //temp_sensor_get_temp(&temperature_high, &temperature_low);

    //uint16_t current_temp = temp_sensor_get_temp();
    //uint8_t temperature_high = HI_BYTE(current_temp);
    //uint8_t temperature_low = LO_BYTE(current_temp);

    if (temp_low_current_level > set_daily_temp_daily_high)
        set_daily_temp_daily_high = temp_low_current_level;
    if (temp_low_current_level < set_daily_temp_daily_low)
        set_daily_temp_daily_low = temp_low_current_level;

    // Is it between 7am and 9am?
    const uint16_t seven_am = 12600; // 60 * 60 * 7 / 2
    const uint16_t nine_am = 16200; // 60 * 60 * 9 / 2
    if (fathom_time_of_day > (seven_am + postpone_set_daily_by) && fathom_time_of_day < (nine_am + postpone_set_daily_by)) {
        // Has it been at least 5hrs since the last time we sent daily temp?
        const uint16_t five_hours = 9000; // 60 * 60 * 5 / 2
        if (fathom_time_of_day - set_daily_temp_last_sent > five_hours) {
            set_daily_temp_last_sent = fathom_time_of_day;

            /*
            // Don't send until after the first day has elapsed
            if (!has_first_day_elasped) {
                has_first_day_elasped = true;

                // Reset these for the next day
                set_daily_temp_daily_high = 0;
                set_daily_temp_daily_low = UINT16_MAX;

                return false;
            }
            */

            return wake_up_sara(STATE_WRITE_SET_DAILY_DELAY);
        }
    }

    return false;
}


/*
bool send_if_needed_gps(void) {
    // TODO: just skip for now
    return false;

    if (power_down_cnt_set_gps)
        return false;

    power_down_cnt_set_gps = true;

    return wake_up_sara(STATE_WRITE_SET_GPS_DELAY);
}
*/


bool send_if_needed_heartbeat(void) {
    if (!send_if_needed_heartbeat_timer_fired)
        return false;

    send_if_needed_heartbeat_timer_fired = false;
    return wake_up_sara(STATE_WRITE_HEARTBEAT_DELAY);
}


/*
  19.2mA with SARA sleeping
  18.3mA with mcu in power-down mode (but usart enabled)
  700uA with usart_disable
  151uA with usart_disable and mcu in power-down mode
 */
void wdt_power_down(void) {

    /*
    UCSRB = 0x00;

    DDRB &= ~(1 << PB2); // Configure RTS as input
    PORTB &= ~(1 << PB2); // RTS down

    // TXD draws 18mA!
    DDRB &= ~(1 << PB0); // Configure TXD as input
    PORTB &= ~(1 << PB0); // TXD down
    */

    /*
    //UCSRB &= ~(1 << RXEN) & ~(1 << TXEN);
    UCSRB = 0x00;
    UCSRC = 0x06;
    UCSRA = 0x20;
    UBRRH = 0x00;
    UBRRL = 0x00;

    GIMSK = 0x00;
    PCMSK1 = 0x00;
    PCMSK2 = 0x00;

    //temp_sensor_set_reg(0x01, 0x60); // 0000 0001 - Configuration register
                                     // 0110 0001 - set R1 and R0 - 12 bits (0.0625 C)

    //DDRD &= ~(1 << PD6); // Configure SCL as input
    //DDRB &= ~(1 << PB2); // Configure SDA as input

    //cli();

    // PRR
    //power_all_disable();
    //PRR |= (1 << PRTIM1); // PRTIM1 up
    //PRR |= (1 << PRTIM0); // PRTIM0 up
    //PRR |= (1 << PRUSI); // PRUSI up
    //PRR |= (1 << PRUSART); // PRUSART up

    // Disable all pull-ups in all ports
    //MCUCR |= (1 << PUD); // PUD up

    DDRD &= ~(1 << PD5); // Configure PWR_ON as input
    PORTD &= ~(1 << PD5); // PWR_ON down

    DDRB &= ~(1 << PB2); // Configure RTS as input
    PORTB &= ~(1 << PB2); // RTS down

    DDRB &= ~(1 << PB0); // Configure TXD as input
    PORTB &= ~(1 << PB0); // TXD down

    DDRD &= ~(1 << PD4); // Configure LED as input
    PORTD &= ~(1 << PD4); // LED down

    DDRB &= ~(1 << PB4); // Configure ETRODE_SUPPLY as input
    PORTB &= ~(1 << PB4); // ETRODE_SUPPLY down

    // may want to disable Brown-out Detector, just to be safe

    // Disable Analog Comparator
    ACSR |= (1 << ACD); // ACD up
    */



    // Enable watchdog timer
    //wdt_enable(WDTO_8S);
    wdt_enable(WDTO_2S); // WDTO_2S is measured to be 2.075s
    WDTCSR |= (1 << WDIE); // WDIE up

    // Go to sleep in Power-down mode
    MCUCR |= (1 << SE); // SE up
    sleep_mode();
}


void wdt_power_down_8s(void) {
    // Enable watchdog timer
    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE); // WDIE up

    // Go to sleep in Power-down mode
    MCUCR |= (1 << SE); // SE up
    sleep_mode();
}


void wdt_upon_wakeup(void) {
    /* 4313:
    // Turn off watchdog timer
    wdt_reset(); // 'wdr' asm

    // Clear WDRF in MCUSR
    //MCUSR = 0x00;
    MCUSR &= ~(1 << WDRF); // WDRF down

    // Write logical one to WDCE and WDE
    //WDTCSR |= (1 << WDCE) | (1 << WDE); // TODO: WDCE is not defined for 1634

    // Turn off WDT
    //WDTCSR = 0x00;
    WDTCSR &= ~(1 << WDE); // WDE down
    */

    // 1634:
    // To disable an enabled Watchdog Timer, the following procedure must be followed:
    // a. Write the signature for change enable of protected I/O registers to register CCP
    // b. Within four instruction cycles, in the same operation, write WDE and WDP bits
    wdt_disable();
    WDTCSR &= ~(1 << WDIE); // WDIE down

    // TODO: is WDRF set by this? if so, we should clear it


    // Enable Analog Comparator
    //ACSR &= ~(1 << ACD); // ACD down
}


// max cnt = 255 (8.5hrs)
void power_down_2min(uint8_t cnt) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    for (uint8_t i = 0; i < cnt; i++) {
        // 15 * 8s = 2min
        for (uint8_t j = 0; j < 15; j++) {
            wdt_power_down_8s();
            wdt_upon_wakeup();
        }
    }
}


// Use the WDT to reset/reboot the AVR
void wdt_reset_avr(void) {
    wdt_enable(WDTO_8S); // measured to be about 7.8s
    for (;;) {
    }
}


uint16_t seconds_to_power_down_cnt(uint16_t seconds) {
    uint16_t ret = seconds / 2;

    // subtract 10%
    //ret -= ret / 10;
    return ret;
}


uint8_t parse_uusord_for_length(uint8_t *read_buf, uint8_t read_buf_len) {
    // +UUSORD: 0,26
    // +UUSORD: 0,27
    uint8_t len = 0;

    uint8_t copy[16] = { 0 };
    uint8_t bytes_to_copy;
    if (read_buf_len > 15)
        bytes_to_copy = 15;
    else
        bytes_to_copy = read_buf_len;
    memcpy(copy, read_buf, bytes_to_copy);

    for (char *p = (char *)copy; *p; ++p) {
        if (*p == ',') {
            len = atoi(p + 1);
            break;
        }
    }

    return len;
}


void adjust_power_down_cnts(void) {
    // tmp
    // 1800
    // 1800
    //eeprom_write_word((void *)(64 + 1*4), heartbeat_interval);
    // end tmp

    // interval is 12hr, so 7am + 12 = 19(7pm)
    // time is 10hr
    // cnt should be 10 - 7 = 3
    //power_down_cnt_heartbeat = fathom_time_of_day - seven_am;

    // interval is 12hr, so 7am + 12 = 19(7pm)
    // time is 14hr
    // cnt should be 14 - 7 = 7
    //power_down_cnt_heartbeat = fathom_time_of_day - seven_am;

    // interval is 12hr, so 7am + 12 = 19(7pm)
    // time is 5hr
    // cnt should be 7 - 5 = 2
    //power_down_cnt_heartbeat = seven_am - fathom_time_of_day;

    // interval is 12hr, so 7am + 12 = 19(7pm)
    // time is 23.5hr
    // cnt should be 23.5 - 19 = 4.5
    //power_down_cnt_heartbeat = 

    const uint16_t seven_am = 12600; // 60 * 60 * 7 / 2
    const uint16_t seven_pm = 34200; // 60 * 60 * 19 / 2
    if (fathom_time_of_day < (seven_am + postpone_set_daily_by))
        power_down_cnt_heartbeat = (seven_am + postpone_set_daily_by) - fathom_time_of_day;
    else if (fathom_time_of_day > (seven_pm + postpone_set_daily_by))
        power_down_cnt_heartbeat = fathom_time_of_day - (seven_pm + postpone_set_daily_by);
    else
        power_down_cnt_heartbeat = fathom_time_of_day - (seven_am + postpone_set_daily_by);

    // tmp
    // 10551
    // 11230
    //eeprom_write_word((void *)(64 + 2*4), power_down_cnt_heartbeat);
    // end tmp

    // power_down_cnt_heartbeat = 92
    // heartbeat_interval = 10
    while (power_down_cnt_heartbeat >= heartbeat_interval)
        power_down_cnt_heartbeat -= heartbeat_interval;

    // tmp
    // 1551
    // 430
    //eeprom_write_word((void *)(64 + 3*4), power_down_cnt_heartbeat);
    // end tmp

    // Consider setting other power_down_cnt to 0?
}


bool parse_usorf_for_payload(uint8_t *read_buf, uint8_t read_buf_len, uint8_t payload_len, bool benign) {
    // +USORF: 0,"195.34.89.241",13,26,"16 FEB 2018 00:04:33 CET\r\n"\r\n\r\nOK\r\n
    enum {
        STATE_INIT,
        STATE_FOUND_COMMAS,
        STATE_FOUND_OPEN_PAREN,
    };

    uint8_t state = STATE_INIT;
    uint8_t commas = 0;
    uint8_t *read_buf_end = read_buf + read_buf_len;

    for (uint8_t *p = read_buf; p != read_buf_end; ++p) {
        switch (state) {
        case STATE_INIT:
            if (*p == ',') {
                ++commas;
                if (commas == 4)
                    state = STATE_FOUND_COMMAS;
            }
            break;
        case STATE_FOUND_COMMAS:
            if (*p != '"')
                return false;
            state = STATE_FOUND_OPEN_PAREN;
            break;
        case STATE_FOUND_OPEN_PAREN:
            if (p - read_buf + payload_len < read_buf_len) {
                if (pht_fathom_payload_parse(&fathom_payload, p, payload_len)) {
                    if (!benign) {
                        if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_FIRMWARE_PAGE) {
                            // Copy the firmware page so we can finish reading from sara before processing
                            memcpy(firmware_page_copy, fathom_payload.firmware, 64);
                            firmware_page_num_last_received = fathom_payload.firmware_page_num;
                        } else if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_CONFIG) {
                            // TODO: change pht_fathom_payload.h so these names match
                            batt_low_interval = seconds_to_power_down_cnt(60);
                            batt_low_threshold = fathom_payload.config_batt_low_threshold;

                            water_low_interval = seconds_to_power_down_cnt(fathom_payload.config_check_sensor_interval);
                            water_low_threshold = fathom_payload.config_electrode_threshold;

                            temp_low_interval = seconds_to_power_down_cnt(60);
                            temp_low_threshold = fathom_payload.config_temperature_threshold;

                            //set_current_temp_interval = seconds_to_power_down_cnt(fathom_payload.config_send_temperature_interval);

                            fathom_days_offset = fathom_payload.config_days_offset;
                            fathom_time_of_day = fathom_payload.config_day_offset;

                            heartbeat_interval = seconds_to_power_down_cnt(fathom_payload.config_heartbeat_interval);

                            adjust_power_down_cnts();

                            timer0_start_keeping_time();
                        } else if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_DAILY_ACK) {
                            fathom_days_offset = fathom_payload.config_days_offset;
                            fathom_time_of_day = fathom_payload.config_day_offset;

                            adjust_power_down_cnts();
                        }
                    }

                    return true;
                }
            }

            return false;
        }
    }

    return false;
}


uint8_t parse_signal_quality_inner(uint8_t *read_buf, uint8_t read_buf_len, int commas_target) {
    enum {
        STATE_INIT,
        STATE_FOUND_COMMAS,
        STATE_FOUND_DIGIT1,
        STATE_FOUND_DIGIT2,
        STATE_FOUND_DIGIT3
    };

    uint8_t state = STATE_INIT;
    uint8_t commas = 0;
    uint8_t *read_buf_end = read_buf + read_buf_len;
    uint8_t digit1;
    uint8_t digit2;
    uint8_t digit3;
    bool done = false;

    // +CESQ: 99,99,255,255,16,22
    for (uint8_t *p = read_buf; p != read_buf_end; ++p) {
        switch (state) {
        case STATE_INIT:
            if (*p == ',') {
                ++commas;
                if (commas == commas_target)
                    state = STATE_FOUND_COMMAS;
            }
            
            break;
        case STATE_FOUND_COMMAS:
            digit1 = *p;
            if (digit1 >= '0' && digit1 <= '9')
                state = STATE_FOUND_DIGIT1;
            else
                return 255;
            break;
        case STATE_FOUND_DIGIT1:
            digit2 = *p;
            if (digit2 >= '0' && digit2 <= '9')
                state = STATE_FOUND_DIGIT2;
            else
                done = true;
            break;
        case STATE_FOUND_DIGIT2:
            digit3 = *p;
            if (digit3 >= '0' && digit3 <= '9')
                state = STATE_FOUND_DIGIT3;
            done = true;
            break;
        case STATE_FOUND_DIGIT3:
            break;
        }

        if (done)
            break;
    }

    uint8_t ones;
    uint8_t tens;

    switch (state) {
    case STATE_INIT:
        // We didn't find 5 commas
        break;
    case STATE_FOUND_COMMAS:
        // We didn't find any digits
        break;
    case STATE_FOUND_DIGIT1:
        ones = digit1 - 48;
        return ones;
    case STATE_FOUND_DIGIT2:
        tens = digit1 - 48;
        ones = digit2 - 48;
        return tens * 10 + ones;
    case STATE_FOUND_DIGIT3:
        // Valid values are 0-97,255 - so if we found 3 digits, they must be 255, otherwise invalid, which would still be 255
        break;
    }

    return 255;
}


void parse_signal_quality(uint8_t *read_buf, uint8_t read_buf_len) {
    // +CESQ: 99,99,255,255,16,22
    if (read_buf[0] == '+' &&
        read_buf[1] == 'C' &&
        read_buf[2] == 'E' &&
        read_buf[3] == 'S' &&
        read_buf[4] == 'Q' &&
        read_buf[5] == ':' &&
        read_buf[6] == ' ') {
    } else {
        signal_strength_rsrq = 255; // invalid value
        signal_strength_rsrp = 255; // invalid value
        return;
    }

    signal_strength_rsrq = parse_signal_quality_inner(read_buf, read_buf_len, 4);
    signal_strength_rsrp = parse_signal_quality_inner(read_buf, read_buf_len, 5);
}


/*
void test_parse_signal_quality(void) {
    led_off();
    _delay_ms(2000);

    bool failed = false;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,16,22", sizeof("+CESQ: 99,99,255,255,16,22") - 1);
    if (signal_strength_rsrq != 16 || signal_strength_rsrp != 22)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,255,15", sizeof("+CESQ: 99,99,255,255,255,15") - 1);
    if (signal_strength_rsrq != 255 || signal_strength_rsrp != 15)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,0,24", sizeof("+CESQ: 99,99,255,255,0,24") - 1);
    if (signal_strength_rsrq != 0 || signal_strength_rsrp != 24)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,9", sizeof("+CESQ: 99,99,255,255,18,9") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 9)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,0", sizeof("+CESQ: 99,99,255,255,18,0") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 0)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,100", sizeof("+CESQ: 99,99,255,255,18,100") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,255", sizeof("+CESQ: 99,99,255,255,18,255") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,1555", sizeof("+CESQ: 99,99,255,255,18,1555") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,155,", sizeof("+CESQ: 99,99,255,255,18,155,") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,055,", sizeof("+CESQ: 99,99,255,255,18,055,") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,55,", sizeof("+CESQ: 99,99,255,255,18,55,") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 55)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18,", sizeof("+CESQ: 99,99,255,255,18,") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,18", sizeof("+CESQ: 99,99,255,255,18") - 1);
    if (signal_strength_rsrq != 18 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,9,8", sizeof("+CESQ: 99,99,255,255,9,8") - 1);
    if (signal_strength_rsrq != 9 || signal_strength_rsrp != 8)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,0,8", sizeof("+CESQ: 99,99,255,255,0,8") - 1);
    if (signal_strength_rsrq != 0 || signal_strength_rsrp != 8)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,1,8", sizeof("+CESQ: 99,99,255,255,1,8") - 1);
    if (signal_strength_rsrq != 1 || signal_strength_rsrp != 8)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,155,8", sizeof("+CESQ: 99,99,255,255,155,8") - 1);
    if (signal_strength_rsrq != 255 || signal_strength_rsrp != 8)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,,", sizeof("+CESQ: 99,99,255,255,,") - 1);
    if (signal_strength_rsrq != 255 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255,", sizeof("+CESQ: 99,99,255,255,") - 1);
    if (signal_strength_rsrq != 255 || signal_strength_rsrp != 255)
        failed = true;

    parse_signal_quality((uint8_t *)"+CESQ: 99,99,255,255", sizeof("+CESQ: 99,99,255,255") - 1);
    if (signal_strength_rsrq != 255 || signal_strength_rsrp != 255)
        failed = true;

    if (failed) {
        for (;;) {
            led_on();
            _delay_ms(100);
            led_off();
            _delay_ms(100);
        }
    } else {
        // All tests passed, flash once
        led_on();
        _delay_ms(100);
        led_off();
        _delay_ms(100);

        for (;;) {
        }
    }
}
*/

/*
// not tested
void parse_network_name(uint8_t *read_buf) {
    // +CSQ: 15,0\r\n
    // +UCGOPS: 0,0,"T-Mobile"\r\n
    if (read_buf[0] == '+' &&
        read_buf[1] == 'U' &&
        read_buf[2] == 'C' &&
        read_buf[3] == 'G' &&
        read_buf[4] == 'O' &&
        read_buf[5] == 'P' &&
        read_buf[6] == 'S' &&
        read_buf[7] == ':' &&
        read_buf[8] == ' ' &&
        read_buf[9] == '0' &&
        read_buf[10] == ',' &&
        read_buf[11] == '0' &&
        read_buf[12] == ',' &&
        read_buf[13] == '"') {

        char *p = network_name;
        for (uint8_t i = 14; i < 14 + sizeof(network_name) - 1; ++i) {
            if (read_buf[i] == '"')
                break;
            *p = read_buf[i];
        }
    }
}
*/


// from <avr/power.h>
typedef enum {
    clock_div_1 = 0,
    clock_div_2 = 1,
    clock_div_4 = 2,
    clock_div_8 = 3,
    clock_div_16 = 4,
    clock_div_32 = 5,
    clock_div_64 = 6,
    clock_div_128 = 7,
    clock_div_256 = 8
} clock_div_t;

static __inline__ void clock_prescale_set(clock_div_t) __attribute__((__always_inline__));

// Write the signature for change enable of protected I/O register to register CCP.
void clock_prescale_set(clock_div_t __x) {
    uint8_t __tmp = 0xD8;
    __asm__ __volatile__ (
        "in __tmp_reg__,__SREG__" "\n\t"
        "cli" "\n\t"
        "out %1, %0" "\n\t"
        "out %2, %3" "\n\t"
        "out __SREG__, __tmp_reg__"
        : /* no outputs */
        : "d" (__tmp),
          "I" (_SFR_IO_ADDR(CCP)),
          "I" (_SFR_IO_ADDR(CLKPR)),
          "d" (__x)
        : "r16");
}


void read_apn_from_eeprom(void) {
    const uint16_t sim_card_apn_base_address = 3;
    char sim_card_apn[12];

    eeprom_read_block(sim_card_apn, (void *)sim_card_apn_base_address, sizeof(sim_card_apn));
    if (sim_card_apn[0] == 0 || sim_card_apn[0] == 0xff || sim_card_apn[11] != '\0') {
        memset(sim_card_apn, 0, sizeof(sim_card_apn));
        memcpy(sim_card_apn, "m2mglobal", sizeof("m2mglobal") - 1);

        eeprom_write_block(sim_card_apn, (void *)sim_card_apn_base_address, sizeof(sim_card_apn));
    }

    memcpy(data_for_set_apn_cmd, "AT+CGDCONT=1,\"IP\",\"", sizeof("AT+CGDCONT=1,\"IP\",\"") - 1);
    uint8_t sim_card_apn_len = strlen(sim_card_apn);
    memcpy(data_for_set_apn_cmd + sizeof("AT+CGDCONT=1,\"IP\",\"") - 1, sim_card_apn, sim_card_apn_len);
    memcpy(data_for_set_apn_cmd + sizeof("AT+CGDCONT=1,\"IP\",\"") - 1 + sim_card_apn_len, "\"\r", sizeof("\"\r")); // we want the final '\0' written
}


// performance optimization: we should turn SARA off, but only if we know we didn't already turn it off with CPWROFF, etc
void reset_with_error(uint8_t error_code) {
    error_codes_store_code(error_code);
    
    led_off();
    usart_disable();

    // If SARA is on, this will leave it on.
    // If SARA is off, this will turn it on.
    pwr_on_pin_pull_low();

    _delay_ms(3000);

    // We know SARA is now on, this will turn it off.
    pwr_on_pin_pull_low_long();

    power_down_2min(1);
    wdt_reset_avr();
}


void push_c_onto_read_buf(uint8_t c, uint8_t **read_buf_p) {
    **read_buf_p = c;

    // Check for read_buf overrun
    if (*read_buf_p - read_buf > sizeof(read_buf) - 1)
        reset_with_error(ERROR_CODE_READ_BUF_OVERRUN);

    ++(*read_buf_p);
}


int main(void) {
    /*
    // tmp
    uint8_t len;

    len = parse_uusord_for_length((uint8_t *)"+UUSORD: 0,26", sizeof("+UUSORD: 0,26") - 1);
    if (len != 26) {
        for (;;) {
        }
    }

    len = parse_uusord_for_length((uint8_t *)"+UUSORD: 0,27", sizeof("+UUSORD: 0,27") - 1);
    if (len != 27) {
        for (;;) {
        }
    }

    for (;;) {
    }
    // end tmp
    */

    /*
    // tmp
    bool ret;
    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"34.233.140.210\",6133,14,\"\x01\x03\x01\x174\x137\x06\x00\x00\x00\"\r",
                                      sizeof("") - 1, 2, false);

    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"34.233.140.210\",6133,2,\"\x01\n\"\r",
                                      sizeof("+USORF: 0,\"34.233.140.210\",6133,2,\"\x01\n\"\r") - 1, 2, false);
    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"34.233.140.210\",6133,2,\"\x01",
                                      sizeof("+USORF: 0,\"34.233.140.210\",6133,2,\"\x01") - 1, 2, false);
    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r\n\"",
                                      sizeof("+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r\n\"") - 1, 26, false);
    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r\n",
                                      sizeof("+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r\n") - 1, 26, false);
    ret = parse_usorf_for_payload((uint8_t *)"+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r",
                                      sizeof("+USORF: 0,\"195.34.89.241\",13,26,\"16 FEB 2018 00:04:33 CET\r") - 1, 26, false);
    for (;;) {
    }
    // end tmp
    */

    bool we_were_reset_by_wdt = false;
    if (MCUSR & (1 << WDRF))
        we_were_reset_by_wdt = true;

    // In case we were reset by WDT, need to clean up a bit
    MCUSR &= ~(1 << WDRF); // WDRF down
    wdt_disable();

    // Set Clock Division Factor to 4 (7.3728MHz / 4 = 1.8432MHz)
    clock_prescale_set(clock_div_4);

    if (sizeof(ErrorCode) != 5) {
        for (;;) {
        }
    }

    // tmp
    /*
    volatile uint16_t trap_interval_test;
    trap_interval_test = seconds_to_power_down_cnt(60); // 27
    trap_interval_test = seconds_to_power_down_cnt(2); // 1
    trap_interval_test = seconds_to_power_down_cnt(4); // 2
    trap_interval_test = seconds_to_power_down_cnt(8); // 4
    trap_interval_test = seconds_to_power_down_cnt(16); // 8
    trap_interval_test = seconds_to_power_down_cnt(18); // 9
    trap_interval_test = seconds_to_power_down_cnt(20); // 9
    trap_interval_test = seconds_to_power_down_cnt(22); // 10
    trap_interval_test = seconds_to_power_down_cnt(24); // 11
    if (trap_interval_test != 11) {
        for (;;) {
        }
    }
    */
    // end tmp

    //_delay_ms(500);

    // Instead of going from high to low, PWR_ON should go from input to output (low)
    // This may have been damaging the SARA!
    //PORTB |= (1 << PB3); // PWR_ON up
    //DDRB |= (1 << PB3); // Configure PWR_ON as output
    PORTB &= ~(1 << PB3); // PWR_ON down
    DDRB &= ~(1 << PB3); // Configure PWR_ON as input

    PORTB |= (1 << PB2); // RTS up
    DDRB |= (1 << PB2); // Configure RTS as output

    DDRC &= ~(1 << PC0); // Configure CTS as input

    PORTB |= (1 << PB0); // TXD up
    DDRB |= (1 << PB0); // Configure TXD as output

    DDRA &= ~(1 << PA7); // Configure RXD as input

    // Read fathom_id from eeprom
    eeprom_read_block(&fathom_id, (void *)fathom_id_base_address, sizeof(fathom_id));
    /*
    if (fathom_id == 0) {
        for (;;) {
        }
    }
    */

    read_apn_from_eeprom();

    // Want to stagger SET_DAILY so they don't all report in at once
    // but the daily high and low should be snapshotted at 7am every morning, but then sent staggered
    // Calc postpone_set_daily_by from fathom_id
    postpone_set_daily_by = (fathom_id * 5) % 1999; // at most 66.6mins

    read_power_down_after_deactivate_pdp_ctx_cnt(we_were_reset_by_wdt);

    error_codes_setup();

    /*
    // tmp
    error_codes_store_code(0);
    error_codes_store_code(1);
    error_codes_store_code(2);
    error_codes_setup();
    error_codes_store_code(3);
    error_codes_store_code(4);
    error_codes_store_code(5);
    error_codes_store_code(6);
    error_codes_store_code(7);
    error_codes_store_code(8);
    error_codes_setup();
    error_codes_store_code(9);
    error_codes_setup();
    error_codes_store_code(10);
    error_codes_setup();
    error_codes_store_code(11);
    for (;;) {
    }
    // end tmp
    */
    enable_pullups_on_unused_pins();
    led_setup();
    timer0_setup();
    timer1_setup();
    external_eeprom_setup();
    adc_setup();
    batt_low_setup();
    water_low_setup();
    temp_sensor_setup();
    temp_low_setup();

    // tmp
    /*
    srand(fathom_id);
    int rnd = rand();
    if (rnd % 2) {
        led_on();
    }
    */
    // end tmp

    // The first ADC conversion result after switching reference voltage source may be inaccurate, and the user is advised to discard this result
    batt_low_get_level();

    led_on();

    /* don't need to do this here as long as we are certain
    // If SARA is on (due to wdt_reset_avr, etc), this will turn it off.
    // If SARA is off (due to AT+CPWROFF), this will turn it on.
    pwr_on_pin_pull_low_long();

    _delay_ms(1500);

    // If SARA is on, this will leave it on.
    // If SARA is off, this will turn it on.
    pwr_on_pin_pull_low();
    */

    // If SARA is off (due to AT+CPWROFF or pwr_on_pin_pull_low_long), this will turn it on.
    // If SARA is on, this will leave it on.
    pwr_on_pin_pull_low();

    _delay_ms(4000); // Need to wait 4s for SARA to be fully init'd

    /*
    if (!temp_sensor_configure()) {
        error_codes_store_code(ERROR_CODE_TEMP_SENSOR_CONFIGURE_FAILED);

        // The theory here is that if the temp_sensor fails, we should continue to monitor the water level.
        // Upon the next SET_CURRENT_TEMP, the server will know the temp_sensor failed, and can notify the farmer (and us).
        temp_sensor_configured = false;
    }
    */

    external_eeprom_test();

    // Power-down when sleeping
    //MCUCR |= (1 << SM1); // SM1 up
    //MCUCR &= ~(1 << SM0); // SM0 down
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    set_sleep_mode(SLEEP_MODE_IDLE); // This allows timer0 to continue running

    // Set Watchdog Timer Prescaler (9.5s)
    //WDTCR |= (1 << WDP3); // WDP3 up
    //WDTCR &= ~(1 << WDP2); // WDP2 down
    //WDTCR &= ~(1 << WDP1); // WDP1 down
    //WDTCR |= (1 << WDP0); // WDP0 up
    //WDTCR = 0x21; // 0010 0001

    // USART
    usart_enable();

    // Set frame format: 8N1 (8 data bits, no parity, 1 stop bit)
    UCSR0C = (3 << UCSZ00);

    sei(); // enable interrupts

    // tmp
    //test_parse_signal_quality();
    // end tmp

    // Tell SARA we are ready to receive
    PORTB &= ~(1 << PB2); // RTS down

    //_delay_ms(10);
    _delay_ms(2000);

    static bool frame_error;
    static bool data_overrun;
    static bool parity_error;

    cmd = &(cmds[cmds_index]);
    cmd_p = cmd->cmd;

    static char *p_data_for_sendto_cmd = data_for_sendto_cmd;
    static uint8_t *p_data_for_sendto_data_cmd = data_for_sendto_data_cmd;
    static char *p_data_for_receive_from_cmd = data_for_receive_from_cmd;
    static char *p_data_for_set_apn_cmd = data_for_set_apn_cmd;

    enum {
        STATE_READ_AWAITING_CMD,
        STATE_READ_GOT_CMD,
        STATE_READ_GOT_EXPECT_LINE1,
        STATE_READ_GOT_EXPECT_LINE2,
        STATE_READ_GOT_EXPECT_LINE3,
    };

    static uint8_t state_read = STATE_READ_AWAITING_CMD;
    state_write = STATE_WRITE_CMD;

    // max response size: 13 chars
    // max response size: 31 chars (AT+CIND?)
    // AT+USOST: 43 + 20 (length of packet) = 64
    //uint8_t read_buf[64];
    //static uint8_t read_buf[78]; // making this static means it shows up in AtmelStudio's 'Data Memory Usage'
    // read_buf can be extended to 110, so we can send 64 bytes at a time:
       // 43 + 2 address bytes (256 pages) + 64 bytes  = 109 (round to 110)
    //static uint8_t read_buf[110]; // making this static means it shows up in AtmelStudio's 'Data Memory Usage'
    static uint8_t *read_buf_p = read_buf;
    static bool udp_packet_ready_to_be_read = false;
    static uint8_t udp_packet_ready_to_be_read_len = 0;
    static bool do_not_wait_for_udp_packet = false;

    // tmp
    /*
    for (uint8_t i = 0; i < 10; ++i) {
        _delay_ms(1500);
        led_off();
        //electrode_level_current_level = electrode_level_get_level();
        electrode_level_current_level = batt_low_get_level();
        led_on();

        led_off();
        eeprom_write_word((void *)(error_code_base_address + i*4), electrode_level_current_level);
        led_on();
    }
    */

    /*
    timer0_start_keeping_time();
    for (;;) {
    }
    */
    // end tmp

    // Event loop
    for (;;) {
        /*
        // tmp - brainstorming sara module
        if (UCSR0A & (1 << RXC0)) {
            // Data is available to read
            sara_read();
            if ()
                parse_signal_quality();

        } else if (UCSR0A & (1 << UDRE0)) {
            // Data can be written
            bool done_writing_cmds = sara_write(); // call this even if there's nothing to write
            if (done_writing_cmds) {
                // Power down mcu for 2s
                // ...
                sara_begin_writing(cmds_send_udp_packet);
            }
        }
        // end tmp
        */

        // Check usart - for data to read or write
        if (UCSR0A & (1 << RXC0)) {
            // Data is available to read
            uint8_t c = usart_read(&frame_error, &data_overrun, &parity_error);
            if (frame_error) {
                //reset_with_error(ERROR_CODE_USART_FRAME_ERROR);
                continue;
            } else if (data_overrun) {
                reset_with_error(ERROR_CODE_USART_DATA_OVERRUN);
            } else if (parity_error) {
                reset_with_error(ERROR_CODE_USART_PARITY_ERROR);
            } else {
                if (c == '\n') {
                    // Special case: we may get a '\n' in the payload
                    if (cmd && cmd->cmd == cmd_receive_from_cmd && state_read == STATE_READ_GOT_CMD) {
                        if (!parse_usorf_for_payload(read_buf, read_buf_p - read_buf, udp_packet_ready_to_be_read_len, true)) {
                            push_c_onto_read_buf(c, &read_buf_p);
                            continue;
                        }
                    }

                    // Ignore '\r' at the end
                    if (read_buf_p != read_buf && *(read_buf_p - 1) == '\r')
                        --read_buf_p;

                    uint8_t line_len;
                    switch (state_read) {
                    case STATE_READ_AWAITING_CMD:
                        if (memcmp(read_buf, "+UUSORF: ", sizeof("+UUSORF: ") - 1) == 0) {
                            if (state_write != STATE_WRITE_WAIT_FOR_UDP_PACKET) {
                                // We are not waiting for a udp packet, but we got one. Just ignore.
                                error_codes_store_code(ERROR_CODE_UNEXPECTED_UUSORD);
                                break;
                            }

                            // +UUSORD: 0,26\r\n
                            // For the UDP socket type the URC +UUSORD: <socket>,<length> notifies that a UDP packet has been received,
                            // either when buffer is empty or after a UDP packet has been read and one or more packets are stored in the buffer.
                            udp_packet_ready_to_be_read = true;
                            udp_packet_ready_to_be_read_len = parse_uusord_for_length(read_buf, read_buf_p - read_buf);

                            strcpy(data_for_receive_from_cmd, "AT+USORF=0,");
                            strcat(data_for_receive_from_cmd, itoa_decimal(udp_packet_ready_to_be_read_len));
                            strcat(data_for_receive_from_cmd, "\r");
                        } else if (memcmp(read_buf, "+UUPSDD: ", sizeof("+UUPSDD: ") - 1) == 0) {
                            // +UUPSDD: 0\r\n\r\n
                            // The +UUPSDD URC is raised when the data connection related to the provided PSD profile is deactivated either
                            // explicitly by the network (e.g. due to prolonged idle time) or locally by the module after a failed PS registration
                            // procedure (e.g. due to roaming) or a user required detach (e.g. triggered by AT+COPS=2).

                            error_codes_store_code(ERROR_CODE_UNEXPECTED_UUPSDD);
                        } else if (memcmp(read_buf, "+UUSOCL: ", sizeof("+UUSOCL: ") - 1) == 0) {
                            // +UUSOCL: 0\r\n
                            // +UUSOCL In case of remote socket closure the user is notified via the URC.

                            // This seems to happen when the server sends us an unexpected udp packet when we are
                            // already (or about to begin) deactivating the pdp ctx.
                            // +UUSORD comes first, then when we issue deactivate_pdp_ctx, sara sends us this UUSOCL.

                            // But this also seems to happen normally sometimes and is harmless, so don't log as an error
                            //error_codes_store_code(ERROR_CODE_UNEXPECTED_UUSOCL);
                        } else if (memcmp(read_buf, "+UULOC: ", sizeof("+UULOC: ") - 1) == 0) {
                            // +UULOC: 01/01/2004,00:29:20.000,0.0000000,0.0000000,0,20000000,0,0,0,0,0,0,0
                            // +UULOC: 26/05/2018,20:26:32.000,38.1694876,-79.0766634,0,1214,0,0,0,2,0,0,0
                        } else if (cmd) {
                            // TODO: try replacing the hardcoded string literals below with references to their cmd (eg cmd_reset_cmd).
                            if (cmd->cmd == cmd_reset_cmd) {
                                if (read_buf_p - read_buf == sizeof("AT+CFUN=15\r") - 1 && memcmp(read_buf, "AT+CFUN=15\r", sizeof("AT+CFUN=15\r") - 1) == 0) {
                                    state_read = STATE_READ_GOT_CMD;
                                    break;
                                }
                            } else if (cmd->cmd == cmd_echo_off_cmd) {
                                if (read_buf_p - read_buf == sizeof("ATE0\r") - 1 && memcmp(read_buf, "ATE0\r", sizeof("ATE0\r") - 1) == 0) {
                                    state_read = STATE_READ_GOT_CMD;
                                    break;
                                } else if (read_buf_p == read_buf) {
                                    // In this case we don't get an echo since we had already turned off echo
                                    state_read = STATE_READ_GOT_CMD;
                                    break;
                                }
                            } else {
                                if (read_buf_p == read_buf) {
                                    // We got a blank line, which is what we want since echo is turned off
                                    state_read = STATE_READ_GOT_CMD;
                                    break;
                                }
                            }

                            reset_with_error(ERROR_CODE_STATE_READ_AWAITING_CMD);
                        } else if (read_buf_p - read_buf) {
                            // We got a non-blank line we were not expecting
                            reset_with_error(ERROR_CODE_UNEXPECTED_LINE);
                        }
                        break;
                    case STATE_READ_GOT_CMD:
                        line_len = strlen_P(cmd->expect_line1);
                        if (line_len == 0) {
                            if (cmd->cmd == cmd_signal_qual_cmd) {
                                parse_signal_quality(read_buf, read_buf_p - read_buf);

                                if (cmds == cmds_init) {
                                    if (signal_strength_rsrp != 255) {
                                        // Convert from scale 0-97 to scale 1-5
                                        timer0_blink_cnt = signal_strength_rsrp / 10;
                                        if (timer0_blink_cnt == 0)
                                            timer0_blink_cnt = 1;
                                        else if (timer0_blink_cnt > 5)
                                            timer0_blink_cnt = 5;

                                        timer0_start();
                                    }
                                }
                            } else if (cmd->cmd == cmd_receive_from_cmd) {
                                parse_usorf_for_payload(read_buf, read_buf_p - read_buf, udp_packet_ready_to_be_read_len, false);
                            }

                            if (cmd->expect_line2) {
                                state_read = STATE_READ_GOT_EXPECT_LINE1;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;
                            }
                            break;
                        }

                        if (line_len == (read_buf_p - read_buf) && memcmp_P(read_buf, cmd->expect_line1, line_len) == 0) {
                            if (cmd->expect_line2) {
                                state_read = STATE_READ_GOT_EXPECT_LINE1;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;

                                if (cmd->cmd == cmd_reset_cmd) {
                                    // Calling this just to set baud back to 0
                                    usart_enable();
                                }
                            }
                        } else {
                            // tmp

                            // this doesn't work
                            //eeprom_write_block(cmd->cmd, (void *)64, strlen_P(cmd->cmd));

                            // this doesn't work
                            // write cmd->expect_line1, line_len
                            //eeprom_write_block(cmd->expect_line1, (void *)128, line_len);

                            // this works
                            // write read_buf, read_buf_p - read_buf
                            //eeprom_write_block(read_buf, (void *)192, read_buf_p - read_buf);
                            // end tmp

                            reset_with_error(ERROR_CODE_STATE_READ_GOT_CMD1);
                        }
                        break;
                    case STATE_READ_GOT_EXPECT_LINE1:
                        line_len = strlen_P(cmd->expect_line2);
                        if (line_len == 0) {
                            if (cmd->expect_line3) {
                                state_read = STATE_READ_GOT_EXPECT_LINE2;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;
                            }
                            break;
                        }

                        if (line_len == (read_buf_p - read_buf) && memcmp_P(read_buf, cmd->expect_line2, line_len) == 0) {
                            if (cmd->expect_line3) {
                                state_read = STATE_READ_GOT_EXPECT_LINE2;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;
                            }
                        } else {
                            reset_with_error(ERROR_CODE_STATE_READ_GOT_EXPECT_LINE1);
                        }
                        break;
                    case STATE_READ_GOT_EXPECT_LINE2:
                        line_len = strlen_P(cmd->expect_line3);
                        if (line_len == 0) {
                            if (cmd->expect_line4) {
                                state_read = STATE_READ_GOT_EXPECT_LINE3;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;
                            }
                            break;
                        }

                        if (line_len == (read_buf_p - read_buf) && memcmp_P(read_buf, cmd->expect_line3, line_len) == 0) {
                            if (cmd->cmd == cmd_receive_from_cmd) {
                                if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_FIRMWARE_PAGE) {
                                    // Write fathom_payload.firmware to fathom_payload.firmware_page_num
                                    // takes about 500ms
                                    if (!external_eeprom_page_write(fathom_payload.firmware_page_num, firmware_page_copy)) {
                                        reset_with_error(ERROR_CODE_EXTERNAL_EEPROM_WRITE_FAILED);
                                    }

                                    // Read it back and confirm
                                    // takes about 500ms
                                    if (!external_eeprom_page_verify(fathom_payload.firmware_page_num, firmware_page_copy)) {
                                        reset_with_error(ERROR_CODE_EXTERNAL_EEPROM_VERIFY_FAILED);
                                    }
                                } else if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_UPDATE_FIRMWARE_FROM_EXT_EEPROM) {
                                    bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx_firmware_checksum = fathom_payload.firmware_checksum;
                                    bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx = true;
                                }
                            } else if (cmds == cmds_init && cmd->cmd == cmd_signal_qual_cmd) {
                                state_read = STATE_READ_AWAITING_CMD;

                                if (signal_strength_rsrp != 255) {
                                    // timer0 will change state_write to STATE_WRITE_CMD_DELAY_OVER
                                    state_write = STATE_WRITE_CMD_DELAY;
                                } else {
                                    if (++signal_strength_retry_cnt == signal_strength_retry_threshold_init) {
                                        // We've waited too long, reset

                                        // This is normal when the signal strength is weak.
                                        // In fact, this is our first indication that the signal is too weak to use.
                                        error_codes_store_code(ERROR_CODE_STATE_READ_GOT_CMD2);

                                        set_power_down_after_deactivate_pdp_ctx();
                                        reset_after_deactivate_pdp_ctx = true;

                                        state_read = STATE_READ_AWAITING_CMD;

                                        cmds = cmds_deactivate_pdp_ctx;
                                        cmds_index = 0;
                                        cmd = &(cmds[cmds_index]);
                                        cmd_p = cmd->cmd;
                                        state_write = STATE_WRITE_CMD;
                                    } else {
                                        // Wait a tick and retry
                                        _delay_ms(10);
                                        cmd_p = cmd->cmd;
                                        state_write = STATE_WRITE_CMD;
                                    }
                                }
                                break;
                            } else if (cmds == cmds_create_udp_socket_signal_qual && cmd->cmd == cmd_signal_qual_cmd) {
                                state_read = STATE_READ_AWAITING_CMD;

                                if (signal_strength_rsrp != 255) {
                                    state_write = STATE_WRITE_CMD_WAIT_OVER;

                                    data_for_sendto_data_cmd[4] = signal_strength_rsrp; // signal_strength_rsrp
                                    data_for_sendto_data_cmd[5] = signal_strength_rsrq; // signal_strength_rsrq
                                } else {
                                    if (++signal_strength_retry_cnt == signal_strength_retry_threshold_socket) {
                                        // We've waited too long, don't retry
                                        state_write = STATE_WRITE_CMD_WAIT_OVER;

                                        data_for_sendto_data_cmd[4] = signal_strength_rsrp; // signal_strength_rsrp
                                        data_for_sendto_data_cmd[5] = signal_strength_rsrq; // signal_strength_rsrq
                                    } else {
                                        // Wait a tick and retry
                                        _delay_ms(10);
                                        cmd_p = cmd->cmd;
                                        state_write = STATE_WRITE_CMD;
                                    }
                                }
                                break;
                            }

                            if (cmd->expect_line4) {
                                state_read = STATE_READ_GOT_EXPECT_LINE3;
                            } else {
                                state_read = STATE_READ_AWAITING_CMD;
                                state_write = STATE_WRITE_CMD_WAIT_OVER;
                            }
                        } else {
                            reset_with_error(ERROR_CODE_STATE_READ_GOT_EXPECT_LINE2);
                        }
                        break;
                    case STATE_READ_GOT_EXPECT_LINE3:
                        line_len = strlen_P(cmd->expect_line4);
                        if (line_len == 0) {
                            state_read = STATE_READ_AWAITING_CMD;
                            state_write = STATE_WRITE_CMD_WAIT_OVER;
                            break;
                        }

                        if (line_len == (read_buf_p - read_buf) && memcmp_P(read_buf, cmd->expect_line4, line_len) == 0) {
                            state_read = STATE_READ_AWAITING_CMD;
                            state_write = STATE_WRITE_CMD_WAIT_OVER;
                        } else {
                            reset_with_error(ERROR_CODE_STATE_READ_GOT_EXPECT_LINE3);
                        }
                        break;
                    }

                    read_buf_p = read_buf;
                } else if (c == '@') {
                    if (read_buf_p == read_buf && cmd && cmd->cmd == cmd_sendto_cmd) {
                        state_read = STATE_READ_AWAITING_CMD;
                        state_write = STATE_WRITE_CMD_WAIT_OVER;
                    } else {
                        push_c_onto_read_buf(c, &read_buf_p);
                    }
                } else {
                    if (c == '\r' && cmd && cmd->cmd == cmd_uart_data_rate_cmd && state_read == STATE_READ_AWAITING_CMD) {
                        //uint16_t baud = 2; // 38400 (at U2X0 = 0 and 1.8432MHz) (measured to be ?)
                        //uint16_t baud = 5; // 19200 (at U2X0 = 0 and 1.8432MHz) (measured to be ?)
                        uint16_t baud = 11; // 9600 (at U2X0 = 0 and 1.8432MHz) (measured to be ?)
                        UBRR0H = (uint8_t)(baud >> 8);
                        UBRR0L = (uint8_t)baud;

                        // Continue reading for the next 5ms, discarding everything read (since it contains frame errors).
                        // Then we can start writing the next cmd.
                        for (uint8_t i = 0; i < 250; ++i) {
                            while (UCSR0A & (1 << RXC0))
                                usart_read(&frame_error, &data_overrun, &parity_error);

                            //_delay_us(20);
                            _delay_us(40);
                        }

                        state_read = STATE_READ_AWAITING_CMD;
                        state_write = STATE_WRITE_CMD_WAIT_OVER;
                        read_buf_p = read_buf;
                        continue;
                    }

                    push_c_onto_read_buf(c, &read_buf_p);
                }
            }
        } else if (UCSR0A & (1 << UDRE0)) {
            // Data can be written
            switch (state_write) {
            case STATE_WRITE_CMD:
                if (cmd_p == cmd_sendto_cmd) {
                    usart_write(*p_data_for_sendto_cmd);
                    ++p_data_for_sendto_cmd;
                    if (!*p_data_for_sendto_cmd) {
                        p_data_for_sendto_cmd = data_for_sendto_cmd;
                        state_write = STATE_WRITE_CMD_WAIT;
                    }
                } else if (cmd_p == cmd_sendto_data_cmd) {
                    usart_write(*p_data_for_sendto_data_cmd);
                    ++p_data_for_sendto_data_cmd;
                    if (p_data_for_sendto_data_cmd - data_for_sendto_data_cmd == data_for_sendto_data_cmd_len) {
                        if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_GET_EEPROM) {
                            // We just finished writing the header for GET_EEPROM_ACK, now we need to write the 256 bytes of eeprom data
                            if (get_eeprom_ack_read_address < 256) {
                                --p_data_for_sendto_data_cmd;
                                *p_data_for_sendto_data_cmd = eeprom_read_byte((void *)get_eeprom_ack_read_address);

                                ++get_eeprom_ack_read_address;
                            } else {
                                p_data_for_sendto_data_cmd = data_for_sendto_data_cmd;
                                state_write = STATE_WRITE_CMD_WAIT;
                            }
                        } else {
                            p_data_for_sendto_data_cmd = data_for_sendto_data_cmd;
                            state_write = STATE_WRITE_CMD_WAIT;
                        }
                    }
                } else if (cmd_p == cmd_receive_from_cmd) {
                    usart_write(*p_data_for_receive_from_cmd);
                    ++p_data_for_receive_from_cmd;
                    if (!*p_data_for_receive_from_cmd) {
                        p_data_for_receive_from_cmd = data_for_receive_from_cmd;
                        state_write = STATE_WRITE_CMD_WAIT;
                    }
                } else if (cmd_p == cmd_set_apn_cmd) {
                    usart_write(*p_data_for_set_apn_cmd);
                    ++p_data_for_set_apn_cmd;
                    if (!*p_data_for_set_apn_cmd) {
                        p_data_for_set_apn_cmd = data_for_set_apn_cmd;
                        state_write = STATE_WRITE_CMD_WAIT;
                    }
                } else {
                    usart_write(*cmd_p);
                    ++cmd_p;
                    if (!*cmd_p)
                        state_write = STATE_WRITE_CMD_WAIT;
                }
                break;
            case STATE_WRITE_CMD_WAIT:
                // Just wait here until we are done reading the response

                // If a UDP packet needs to be read, go ahead and read it (this seems to happen only if a
                // PDP context was still active in the SARA when we attempted cmd_reset)
                if (udp_packet_ready_to_be_read) {
                    udp_packet_ready_to_be_read = false;

                    cmds = cmds_receive_from;
                    cmds_index = 0;
                    cmd = &(cmds[cmds_index]);
                    cmd_p = cmd->cmd;
                    state_write = STATE_WRITE_CMD;
                }

                break;
            case STATE_WRITE_CMD_WAIT_OVER:
                if (cmd->delay != 0) {
                    timer1_delay_100ms_cnt = cmd->delay;
                    timer1_start();
                    state_write = STATE_WRITE_CMD_DELAY;
                } else {
                    state_write = STATE_WRITE_CMD_DELAY_OVER;
                }
                break;
            case STATE_WRITE_CMD_DELAY:
                break;
            case STATE_WRITE_CMD_DELAY_OVER:
                timer1_stop();
                cmd = NULL;
                cmd_p = NULL;

                // We have finished writing the cmd, start writing the next cmd in the array
                if (cmds == cmds_init) {
                    if (cmds_index < ARRLEN(cmds_init) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        data_for_sendto_data_cmd_len = 0;
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_INIT; // type
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(firmware_version); // firmware_version low
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(firmware_version); // firmware_version high
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = signal_strength_rsrp; // signal_strength_rsrp
                        data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = signal_strength_rsrq; // signal_strength_rsrq

                        strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                        strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                        strcat(data_for_sendto_cmd, "\r");

                        //led_on(); // leave it off for a while since we just finished blinking
                        wait_for_udp_packet_retries = 0;

                        cmds = cmds_create_udp_socket;
                        cmds_index = 0;
                        cmd = &(cmds[cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    }
                } else if (cmds == cmds_create_udp_socket) {
                    if (cmds_index < ARRLEN(cmds_create_udp_socket) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        cmds = cmds_send_udp_packet;
                        cmds_index = 0;
                        cmd = &(cmds[cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    }
                } else if (cmds == cmds_create_udp_socket_signal_qual) {
                    if (cmds_index < ARRLEN(cmds_create_udp_socket_signal_qual) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        cmds = cmds_send_udp_packet;
                        cmds_index = 0;
                        cmd = &(cmds[cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    }
                } else if (cmds == cmds_send_udp_packet) {
                    if (cmds_index < ARRLEN(cmds_send_udp_packet) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        if (do_not_wait_for_udp_packet) {
                            do_not_wait_for_udp_packet = false;

                            cmds = cmds_deactivate_pdp_ctx;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        } else {
                            wait_for_udp_packet_cnt = 0;
                            state_write = STATE_WRITE_WAIT_FOR_UDP_PACKET;
                        }
                    }
                } else if (cmds == cmds_receive_from) {
                    if (cmds_index < ARRLEN(cmds_receive_from) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_CONFIG) {
                            // We just received SET_CONFIG, so send SET_CONFIG_ACK
                            data_for_sendto_data_cmd_len = 0;
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_SET_CONFIG_ACK; // type
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = fathom_payload.config_version; // config_version

                            // SET_CONFIG may have changed thresholds, so we want to check these and return them with SET_CONFIG_ACK
                            batt_low_most_recent_is_batt_low = batt_low_is_batt_low(true);
                            water_low_most_recent_is_water_low = water_low_is_water_low();

                            // This is defensive to see if we can catch the occasional bug with temp_sensor
                            if (timer1_is_running()) {
                                reset_with_error(ERROR_CODE_TIMER1_RUNNING1);
                            }

                            temp_low_most_recent_is_temp_low = temp_low_is_temp_low();

                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = batt_low_most_recent_is_batt_low; // is_batt_low
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = water_low_most_recent_is_water_low; // is_water_low
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = temp_low_most_recent_is_temp_low; // is_temp_low

                            strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                            strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                            strcat(data_for_sendto_cmd, "\r");

                            wait_for_udp_packet_retries = 0;

                            cmds = cmds_send_udp_packet;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        } else if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_SET_FIRMWARE_PAGE) {
                            // We just received SET_FIRMWARE_PAGE, so send SET_FIRMWARE_PAGE_ACK
                            data_for_sendto_data_cmd_len = 0;
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_SET_FIRMWARE_PAGE_ACK; // type
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high

                            strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                            strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                            strcat(data_for_sendto_cmd, "\r");

                            wait_for_udp_packet_retries = 0;

                            cmds = cmds_send_udp_packet;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        } else if (fathom_payload.type == PHT_FATHOM_PAYLOAD_TYPE_GET_EEPROM) {
                            // We just received GET_EEPROM, so send GET_EEPROM_ACK
                            data_for_sendto_data_cmd_len = 0;
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_GET_EEPROM_ACK; // type
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                            data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high

                            get_eeprom_ack_read_address = 0; // point at starting address

                            strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,260\r"); // 4 + 256 = 260

                            do_not_wait_for_udp_packet = true;

                            cmds = cmds_send_udp_packet;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        } else {
                            cmds = cmds_deactivate_pdp_ctx;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        }
                    }
                } else if (cmds == cmds_deactivate_pdp_ctx) {
                    if (cmds_index < ARRLEN(cmds_deactivate_pdp_ctx) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        led_off();
                        usart_disable();
                        state_write = STATE_WRITE_IDLE;
                    }
                    /*
                } else if (cmds == cmds_uloccell) {
                    if (cmds_index < ARRLEN(cmds_uloccell) - 1) {
                        cmd = &(cmds[++cmds_index]);
                        cmd_p = cmd->cmd;
                        state_write = STATE_WRITE_CMD;
                    } else {
                        state_write = STATE_WRITE_WAIT_FOR_GPS_RESPONSE;
                    }
                    */
                }
                break;
            case STATE_WRITE_WAIT_FOR_UDP_PACKET:
                if (udp_packet_ready_to_be_read) {
                    udp_packet_ready_to_be_read = false;

                    cmds = cmds_receive_from;
                    cmds_index = 0;
                    cmd = &(cmds[cmds_index]);
                    cmd_p = cmd->cmd;
                    state_write = STATE_WRITE_CMD;
                } else {
                    if (++wait_for_udp_packet_cnt == wait_for_udp_packet_threshold) {
                        // We've waited too long to receive the response, re-try
                        if (++wait_for_udp_packet_retries == 3) {
                            // We've tried 3 times, this should be treated just like weak signal for activate_pdp_ctx
                            error_codes_store_code(ERROR_CODE_WAIT_FOR_UDP_PACKET_RETRIES);

                            set_power_down_after_deactivate_pdp_ctx();
                            reset_after_deactivate_pdp_ctx = true;

                            cmds = cmds_deactivate_pdp_ctx;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        } else {
                            // Resend the last packet
                            cmds = cmds_send_udp_packet;
                            cmds_index = 0;
                            cmd = &(cmds[cmds_index]);
                            cmd_p = cmd->cmd;
                            state_write = STATE_WRITE_CMD;
                        }
                    }
                }
                break;
                /*
            case STATE_WRITE_WAIT_FOR_GPS_RESPONSE:
                // Just wait here until we get a response from SARA
                break;
                */
            case STATE_WRITE_IDLE:
                // May not need this check here
                if (udp_packet_ready_to_be_read) {
                    udp_packet_ready_to_be_read = false;

                    cmds = cmds_receive_from;
                    cmds_index = 0;
                    cmd = &(cmds[cmds_index]);
                    cmd_p = cmd->cmd;
                    state_write = STATE_WRITE_CMD;
                } else {
                    /* 2 options:
                       1) *sleep in idle so that timer0 is always running
                       2) after wakeup, call timer0's function
                    */

                    // Power down mcu for 2s (however it will be woken up at each timer0 overflow, so about every 125ms)
                    wdt_power_down();
                    wdt_upon_wakeup();

                    if (power_down_after_deactivate_pdp_ctx) {
                        power_down_after_deactivate_pdp_ctx = false;

                        power_down_2min(power_down_after_deactivate_pdp_ctx_cnt);

                        if (reset_after_deactivate_pdp_ctx) {
                            // An issue happened earlier, but instead of resetting right away, we wanted to first deactivate the pdp ctx.
                            // Now we just finished deactivating the pdp ctx, so reset.
                            wdt_reset_avr();
                        }
                    } else {
                        clear_power_down_after_deactivate_pdp_ctx_cnt();
                    }

                    if (bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx) {
                        bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx = false;

                        // Verify checksum
                        if (!external_eeprom_checksum_verify(bl_update_firmware_from_eeprom_after_deactivate_pdp_ctx_firmware_checksum)) {
                            reset_with_error(ERROR_CODE_EXTERNAL_EEPROM_CHECKSUM_FAILED);
                        }

                        cli(); // disable interrupts
                        bl_update_firmware_from_eeprom(); // never returns
                    }

                    if (batt_low_send_if_needed()) {
                        wake_up_sara(STATE_WRITE_BATT_LOW_DELAY);
                        break;
                    }

                    if (water_low_send_if_needed()) {
                        wake_up_sara(STATE_WRITE_WATER_LOW_DELAY);
                        break;
                    }

                    // This is defensive to see if we can catch the occasional bug with temp_sensor
                    if (timer1_is_running()) {
                        reset_with_error(ERROR_CODE_TIMER1_RUNNING2);
                    }

                    if (temp_low_send_if_needed()) {
                        wake_up_sara(STATE_WRITE_TEMP_LOW_DELAY);
                        break;
                    }

                    //if (send_if_needed_set_current_temp())
                        //break;
                    if (send_if_needed_set_daily())
                        break;
                    //if (send_if_needed_gps())
                        //break;
                    if (send_if_needed_heartbeat())
                        break;
                }
                break;
            case STATE_WRITE_BATT_LOW_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_BATT_LOW_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_BATT_LOW; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = batt_low_most_recent_is_batt_low; // is_batt_low

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                led_on();
                wait_for_udp_packet_retries = 0;

                cmds = cmds_create_udp_socket;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
            case STATE_WRITE_WATER_LOW_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_WATER_LOW_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_WATER_LOW; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = water_low_most_recent_is_water_low; // is_water_low

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                led_on();
                wait_for_udp_packet_retries = 0;

                cmds = cmds_create_udp_socket;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
            case STATE_WRITE_TEMP_LOW_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_TEMP_LOW_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_TEMP_LOW; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = temp_low_most_recent_is_temp_low; // is_temp_low

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                led_on();
                wait_for_udp_packet_retries = 0;

                cmds = cmds_create_udp_socket;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
                /*
            case STATE_WRITE_SET_CURRENT_TEMP_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_SET_CURRENT_TEMP_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_SET_CURRENT_TEMP; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = set_current_temp_temperature_low; // current_temperature low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = set_current_temp_temperature_high; // current_temperature high

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                led_on();
                wait_for_udp_packet_retries = 0;

                cmds = cmds_create_udp_socket;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
                */
            case STATE_WRITE_SET_DAILY_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_SET_DAILY_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_SET_DAILY; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(set_daily_temp_daily_high); // daily_high_temperature low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(set_daily_temp_daily_high); // daily_high_temperature high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(set_daily_temp_daily_low); // daily_low_temperature low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(set_daily_temp_daily_low); // daily_low_temperature high

                // Reset these for the next day
                set_daily_temp_daily_high = 0;
                set_daily_temp_daily_low = UINT16_MAX;

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                led_on();
                wait_for_udp_packet_retries = 0;

                cmds = cmds_create_udp_socket;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
            case STATE_WRITE_HEARTBEAT_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_HEARTBEAT_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                data_for_sendto_data_cmd_len = 0;
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = protocol_version; // version
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = PHT_FATHOM_PAYLOAD_TYPE_HEARTBEAT; // type
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(fathom_id); // fathom_id low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(fathom_id); // fathom_id high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = 0; // signal_strength_rsrp (hasn't been determined yet)
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = 0; // signal_strength_rsrq (hasn't been determined yet)
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(batt_low_current_level); // batt_level low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(batt_low_current_level); // batt_level high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(water_low_current_level); // electrode_level low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(water_low_current_level); // electrode_level high
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = LO_BYTE(temp_low_current_level); // current_temperature low
                data_for_sendto_data_cmd[data_for_sendto_data_cmd_len++] = HI_BYTE(temp_low_current_level); // current_temperature high

                strcpy(data_for_sendto_cmd, "AT+USOST=0,\"34.233.140.210\",6133,");
                strcat(data_for_sendto_cmd, itoa_decimal(data_for_sendto_data_cmd_len));
                strcat(data_for_sendto_cmd, "\r");

                /*
                // tmp
                led_on();
                _delay_ms(500);
                led_off();
                _delay_ms(500);
                // end tmp
                */

                led_on();
                wait_for_udp_packet_retries = 0;
                signal_strength_retry_cnt = 0;

                cmds = cmds_create_udp_socket_signal_qual;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
                /*
            case STATE_WRITE_SET_GPS_DELAY:
                // Just wait here until timer1 is done
                break;
            case STATE_WRITE_SET_GPS_DELAY_OVER:
                timer1_stop();

                // Tell SARA we are ready to receive
                PORTB &= ~(1 << PB2); // RTS down

                cmds = cmds_uloccell;
                cmds_index = 0;
                cmd = &(cmds[cmds_index]);
                cmd_p = cmd->cmd;
                state_write = STATE_WRITE_CMD;
                break;
                */
            }
        }
    }
}


ISR(WDT_vect) {
    // This ISR needs to be here, even though it's empty
}
