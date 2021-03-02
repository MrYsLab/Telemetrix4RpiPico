//
// Created by afy on 2/18/21.f
//

/********************************************************
 * Copyright (c) 2021 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,f
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

/************************** FORWARD REFERENCES ***********************
We define all functions here as extern to provide allow
forward referencing.
**********************************************************************/

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void pwm_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void get_pico_unique_id();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void serial_write(int *buffer, int num_of_bytes_to_send);

extern void led_debug(int blinks, uint delay);

extern void send_debug_info(uint id, uint value);

extern void dht_new();

extern void stop_all_reports();

extern void enable_all_reports();

extern void reset_data();

extern void reset_board();

extern void scan_digital_inputs();

extern void scan_analog_inputs();



/*********************************************************
 *                       COMMAND DEFINES
 ********************************************************/

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table below.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define PWM_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define GET_PICO_UNIQUE_ID  6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define DHT_NEW 14
#define STOP_ALL_REPORTS 15
#define ENABLE_ALL_REPORTS 16
#define RESET_DATA 17
#define RESET_BOARD 18

/*****************************************************
 *                  MESSAGE OFFSETS
 ***************************************************/
// i2c_common
#define I2C_PORT 1
#define I2C_DEVICE_ADDRESS 2 // read and write

// i2c_init
#define I2C_SDA_GPIO_PIN 2
#define I2C_SCL_GPIO_PIN 3

// i2c_read
#define I2C_READ_REGISTER 3
#define I2C_READ_LENGTH 4
#define I2C_READ_NO_STOP_FLAG 5

// I2c_write
#define I2C_WRITE_NUMBER_OF_BYTES 3
#define I2C_WRITE_NO_STOP_FLAG 4
#define I2C_WRITE_BYTES_TO_WRITE 5

// This defines how many bytes there are
// that precede the first byte read position
// in the i2c report message buffer.
#define I2C_READ_DATA_BASE_BYTES 5

// Start of i2c data read within the message buffer
#define I2C_READ_START_OF_DATA 6

// Indicator that no i2c register is being specified in the command
#define I2C_NO_REGISTER 254

/******************************************************
 *                 PIN MODE DEFINITIONS
 *****************************************************/
#define DIGITAL_INPUT 0
#define DIGITAL_OUTPUT 1
#define PWM_OUTPUT 2
#define DIGITAL_INPUT_PULL_UP 3
#define DIGITAL_INPUT_PULL_DOWN 4
#define ANALOG_INPUT 5

#define PIN_MODE_NOT_SET 255

/**************************************************
 *               REPORT DEFINITIONS
 **************************************************/
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT 3
#define FIRMWARE_REPORT 5
#define REPORT_PICO_UNIQUE_ID 6
#define SERVO_UNAVAILABLE 7 // for the future
#define I2C_WRITE_FAILED 8
#define I2C_READ_FAILED 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11 // for the future
#define DEBUG_PRINT 99

/***************************************************************
 *          INPUT PIN REPORTING CONTROL SUB COMMANDS
 ***************************************************************/
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

/*******************************************************************
 *              GLOBAL DEFINES, VARIABLES, AND STORAGE
 ******************************************************************/

const uint LED_PIN = 25; // board LED

/* Maximum Supported pins */
#define MAX_DIGITAL_PINS_SUPPORTED 30
#define MAX_ANALOG_PINS_SUPPORTED 5

/* Firmware Version Values */
#define FIRMWARE_MAJOR 0
#define FIRMWARE_MINOR 2

// Indicator that no i2c register is being specified in the command
#define I2C_NO_REGISTER_SPECIFIED 254

bool stop_reports = false; // a flag to stop sending all report messages

// a descriptor for digital pins
typedef struct {
    uint pin_number;
    uint pin_mode;
    uint reporting_enabled; // If true, then send reports if an input pin
    int last_value;        // Last value read for input mode
} pin_descriptor;

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// a descriptor for analog pins
typedef struct analog_pin_descriptor {
    uint reporting_enabled; // If true, then send reports if an input pin
    int last_value;         // Last value read for input mode
    int differential;       // difference between current and last value needed
} analog_pin_descriptor;

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.
// The command_func is a pointer the command's function.
typedef struct {
    // a pointer to the command processing function
    void (*command_func)(void);
} command_descriptor;

// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

// buffer to hold incoming command data
uint8_t command_buffer[MAX_COMMAND_LENGTH];

// A buffer to hold i2c report data
//int i2c_report_message[64];


/*****************************************************************
 *                   THE COMMAND TABLE
 ****************************************************************/
// An array of pointers to the command functions
command_descriptor command_table[20] =
        {
                {&serial_loopback},
                {&set_pin_mode},
                {&digital_write},
                {&pwm_write},
                {&modify_reporting},
                {&get_firmware_version},
                {&get_pico_unique_id},
                {&servo_attach},
                {&servo_write},
                {&servo_detach},
                {&i2c_begin},
                {&i2c_read},
                {&i2c_write},
                {&sonar_new},
                {&dht_new},
                {&stop_all_reports},
                {&enable_all_reports},
                {&reset_data},
                {&reset_board},
        };

/***************************************************************************
 *                   DEBUGGING FUNCTIONS
 **************************************************************************/

/************************************************************
 * Loop back the received character
 */
void serial_loopback() {
    int loop_back_buffer[3] = {2, (int) SERIAL_LOOP_BACK, command_buffer[1]};
    serial_write(loop_back_buffer, 3);
}

/******************************************************************
 * Send debug info report
 * @param id: 8 bit value
 * @param value: 16 bit value
 */
// A method to send debug data across the serial link
void send_debug_info(uint id, uint value) {
    int debug_buffer[5] = {4, DEBUG_PRINT, 0, 0, 0};
    debug_buffer[2] = id;
    debug_buffer[3] = (value & 0xff00) >> 8;
    debug_buffer[4] = value & 0x00ff;
    serial_write(debug_buffer, 5);
}

/************************************************************
 * Blink the board led
 * @param blinks - number of blinks
 * @param delay - delay in milliseconds
 */
void led_debug(int blinks, uint delay) {
    for (int i = 0; i < blinks; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(delay);
        gpio_put(LED_PIN, 0);
        sleep_ms(delay);
    }
}

/*******************************************************************************
 *                  COMMAND FUNCTIONS
 ******************************************************************************/

/************************************************************************
 * Set a Pins mode
 */
void set_pin_mode() {
    uint pin;
    uint mode;
    //uint16_t range;
    uint range;
    pwm_config pwm_cfg;
    pin = command_buffer[1];
    mode = command_buffer[2];

    switch (mode) {
        case DIGITAL_INPUT:
        case DIGITAL_INPUT_PULL_UP:
        case DIGITAL_INPUT_PULL_DOWN:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[3];
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            if (mode == DIGITAL_INPUT_PULL_UP) {
                gpio_pull_up(pin);
            }
            if (mode == DIGITAL_INPUT_PULL_DOWN) {
                gpio_pull_down(pin);
            }
            break;
        case DIGITAL_OUTPUT:
            the_digital_pins[pin].pin_mode = mode;
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            break;
        case PWM_OUTPUT:
            range = (command_buffer[3] << 8) + command_buffer[4];
            the_digital_pins[pin].pin_mode = mode;
            pwm_cfg = pwm_get_default_config();
            pwm_config_set_wrap(&pwm_cfg, (uint16_t) range);
            pwm_init(pwm_gpio_to_slice_num(pin), &pwm_cfg, true);
            gpio_set_function(pin, GPIO_FUNC_PWM);
            break;

        case ANALOG_INPUT:
            //if the temp sensor was selected, then turn it on
            if (pin == 4) {
                adc_set_temp_sensor_enabled(true);
            }
            the_analog_pins[pin].reporting_enabled = command_buffer[5];
            // save the differential value
            the_analog_pins[pin].differential = (int) ((command_buffer[3] << 8) + command_buffer[4]);
            break;
        default:
            break;
    }
}

/**********************************************************
 * Set a digital output pin's value
 */
void digital_write() {
    uint pin;
    uint value;
    pin = command_buffer[1];
    value = command_buffer[2];
    gpio_put(pin, (bool) value);
}

/**********************************************
 * Set A PWM Pin's value
 */
void pwm_write() {
    uint pin;
    uint value;

    pin = command_buffer[1];
    // the value may be up to 16 bits in length
    value = (command_buffer[2] >> 8) + command_buffer[3];
    pwm_set_gpio_level(pin, (uint16_t) value);
}

/***************************************************
 *  Control reporting
 */
void modify_reporting() {
    int pin = command_buffer[2];

    switch (command_buffer[1]) {
        case REPORTING_DISABLE_ALL:
            for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
                the_digital_pins[i].reporting_enabled = false;
            }
            for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
                the_analog_pins[i].reporting_enabled = false;
            }
            break;
        case REPORTING_ANALOG_ENABLE:
            the_analog_pins[pin].reporting_enabled = true;
            break;
        case REPORTING_ANALOG_DISABLE:
            the_analog_pins[pin].reporting_enabled = false;
            break;
        case REPORTING_DIGITAL_ENABLE:
            if (the_digital_pins[pin].pin_mode != PIN_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_DIGITAL_DISABLE:
            if (the_digital_pins[pin].pin_mode != PIN_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = false;
            }
            break;
        default:
            break;
    }
}

/***********************************************************************
 * Retrieve the current firmware version
 */
void get_firmware_version() {
    int report_message[4] = {3, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR};
    serial_write(report_message, 4);
}

/**************************************************************
 * Retrieve the Pico's Unique ID
 */
void get_pico_unique_id() {
    // get the unique id
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    int report_message[10] = {9, REPORT_PICO_UNIQUE_ID, (board_id.id[0]),
                              board_id.id[1],
                              board_id.id[2],
                              board_id.id[3],
                              board_id.id[4],
                              board_id.id[5],
                              board_id.id[6],
                              board_id.id[7]};

    serial_write(report_message, 10);
}

/********************************************
 * Stop reporting for all input pins
 */
void stop_all_reports() {
    stop_reports = true;
    sleep_ms(20);
    stdio_flush();
}

/**********************************************
 * Enable reporting for all input pins
 */
void enable_all_reports() {
    stdio_flush();
    stop_reports = false;
    sleep_ms(20);
}

/******************************************
 * Use the watchdog time to reset the board.
 */
void reset_board() {
    watchdog_reboot(0, 0, 0);
    watchdog_enable(10, 1);
}

void i2c_begin() {
    // get the GPIO pins associated with this i2c instance
    uint sda_gpio = command_buffer[I2C_SDA_GPIO_PIN];
    uint scl_gpio = command_buffer[I2C_SCL_GPIO_PIN];

    //send_debug_info(command_buffer[I2C_PORT], command_buffer[I2C_SDA_GPIO_PIN]);
    //send_debug_info(command_buffer[I2C_PORT], command_buffer[I2C_SCL_GPIO_PIN]);

    // set the i2c instance - 0 or 1
    if (command_buffer[I2C_PORT] == 0) {
        i2c_init(i2c0, 100 * 1000);
    } else {
        i2c_init(i2c1, 100 * 1000);
    }
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);
}

void i2c_read() {

    // The report_message offsets:
    // 0 = packet length - this must be calculated
    // 1 = I2C_READ_REPORT
    // 2 = The i2c port - 0 or 1
    // 3 = i2c device address
    // 4 = i2c read register
    // 5 = number of bytes read
    // 6... = bytes read

    // Allocate storage for the report.
    // This is a variable amount based on the amount of data
    // to be read.
    int i2c_read_report_message[I2C_READ_DATA_BASE_BYTES + command_buffer[I2C_READ_LENGTH]];
    int num_of_bytes_to_send = I2C_READ_DATA_BASE_BYTES + command_buffer[I2C_READ_LENGTH] + 1;

    // We have a separate buffer ot store the data read from the device
    // and combine that data back into the i2c report buffer.
    // This gets around casting.
    uint8_t data_from_device[command_buffer[I2C_READ_LENGTH]];

    // return value from write and read i2c sdk commands
    int i2c_sdk_call_return_value;

    // selector for i2c0 or i2c1
    i2c_inst_t *i2c;

    // Determine the i2c port to use.
    if (command_buffer[I2C_PORT]) {
        i2c = i2c1;
    } else {
        i2c = i2c0;
    }

    // If there is an i2c register specified, set the register pointer
    if (command_buffer[I2C_READ_NO_STOP_FLAG] != I2C_NO_REGISTER_SPECIFIED) {
        i2c_sdk_call_return_value = i2c_write_blocking(i2c,
                                                       (uint8_t) command_buffer[I2C_DEVICE_ADDRESS],
                                                       (const uint8_t *) &command_buffer[I2C_READ_REGISTER], 1,
                                                       (bool) command_buffer[I2C_READ_NO_STOP_FLAG]);
        if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
            int i2c_error_report_message[5] = {4, I2C_WRITE_FAILED, 2,
                                               command_buffer[I2C_PORT],
                                               command_buffer[I2C_DEVICE_ADDRESS]};
            serial_write(i2c_error_report_message, 5);
            return;
        }
    }
    // now do the read request
    i2c_sdk_call_return_value = i2c_read_blocking(i2c,
                                                  (uint8_t) command_buffer[I2C_DEVICE_ADDRESS],
                                                  data_from_device,
            //data_from_device,
                                                  (size_t) (command_buffer[I2C_READ_LENGTH]),
                                                  (bool) command_buffer[I2C_READ_NO_STOP_FLAG]);
    if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
        int i2c_error_report_message[5] = {4, I2C_READ_FAILED, 2, command_buffer[I2C_PORT],
                                           command_buffer[I2C_DEVICE_ADDRESS]};
        serial_write(i2c_error_report_message, 5);
    }

    // copy the data returned from i2c device into the report message buffer
    for (uint i = 0; i < i2c_sdk_call_return_value; i++) {
        i2c_read_report_message[i + I2C_READ_START_OF_DATA] = data_from_device[i];
    }
    // length of the packet
    i2c_read_report_message[0] = (uint8_t) (i2c_sdk_call_return_value + I2C_READ_DATA_BASE_BYTES);

    i2c_read_report_message[1] = I2C_READ_REPORT;

    // i2c_port
    i2c_read_report_message[2] = command_buffer[I2C_PORT];

    // i2c_address
    i2c_read_report_message[3] = command_buffer[I2C_DEVICE_ADDRESS];

    // i2c register
    i2c_read_report_message[4] = command_buffer[I2C_READ_REGISTER];

    // number of bytes read from i2c device
    i2c_read_report_message[5] = (uint8_t) i2c_sdk_call_return_value;

    serial_write((int *) i2c_read_report_message, num_of_bytes_to_send);

}

void i2c_write() {
    // i2c instance pointer
    i2c_inst_t *i2c;

    // return value from sdk write
    int command_return_value;

    // Determine the i2c port to use.
    if (command_buffer[I2C_PORT]) {
        i2c = i2c1;
    } else {
        i2c = i2c0;
    }

    command_return_value = i2c_write_blocking(i2c, (uint8_t) command_buffer[I2C_DEVICE_ADDRESS],
                                              &(command_buffer[I2C_WRITE_BYTES_TO_WRITE]),
                                              command_buffer[I2C_WRITE_NUMBER_OF_BYTES],
                                              (bool) command_buffer[I2C_WRITE_NO_STOP_FLAG]);

    if (command_return_value == PICO_ERROR_GENERIC) {
        int report_message[4] = {3, I2C_WRITE_FAILED, 1, command_buffer[I2C_PORT]};
        serial_write(report_message, 4);
        return;
    }
}

/******************* FOR FUTURE RELEASES **********************/
void servo_attach() {}

void servo_write() {}

void servo_detach() {}

void sonar_new() {}

void dht_new() {}

void reset_data() {}

/******************************************************
 *             INTERNALLY USED FUNCTIONS
 *****************************************************/

/***************************************************
 * Retrieve the next command and process it
 */
void get_next_command() {
    int packet_size;
    uint8_t packet_data;
    command_descriptor command_entry;

    // clear the command buffer for the new incoming command
    memset(command_buffer, 0, sizeof(command_buffer));

    // Get the number of bytes of the command packet.
    // The first byte is the command ID and the following bytes
    // are the associated data bytes
    if ((packet_size = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT) {
        // no data, let the main loop continue to run to handle inputs
        return;
    } else {
        // get the rest of the packet
        for (int i = 0; i < packet_size; i++) {
            if ((packet_data = (uint8_t) getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT) {
                sleep_ms(1);
            }
            command_buffer[i] = packet_data;
        }
        // the first byte is the command ID.
        // look up the function and execute it.
        // data for the command starts at index 1 in the command_buffer
        command_entry = command_table[command_buffer[0]];

        // uncomment to see the command and first byte of data
        //send_debug_info(command_buffer[0], command_buffer[1]);

        // call the command
        command_entry.command_func();
    }
}

/**************************************
 * Scan all pins set as digital inputs
 * and generate a report.
*/
void scan_digital_inputs() {
    int value;

    // report message

    // index 0 = packet length
    // index 1 = report type
    // index 2 = pin number
    // index 3 = value
    int report_message[4] = {3, DIGITAL_REPORT, 0, 0};

    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        if (the_digital_pins[i].pin_mode == DIGITAL_INPUT ||
            the_digital_pins[i].pin_mode == DIGITAL_INPUT_PULL_UP ||
            the_digital_pins[i].pin_mode == DIGITAL_INPUT_PULL_DOWN) {
            if (the_digital_pins[i].reporting_enabled) {
                // if the value changed since last read
                value = gpio_get(the_digital_pins[i].pin_number);
                if (value != the_digital_pins[i].last_value) {
                    the_digital_pins[i].last_value = value;
                    report_message[2] = i;
                    report_message[3] = value;
                    serial_write(report_message, 4);
                }
            }
        }
    }
}

void scan_analog_inputs() {
    uint16_t value;

    // report message

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = pin number
    // byte 3 = high order byte of value
    // byte 4 = low order byte of value

    int report_message[5] = {4, ANALOG_REPORT, 0, 0, 0};
    int differential;


    for (uint8_t i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        if (the_analog_pins[i].reporting_enabled) {
            adc_select_input(i);
            value = adc_read();
            differential = abs(value - the_analog_pins[i].last_value);
            if (differential >= the_analog_pins[i].differential) {
                //trigger value achieved, send out the report
                the_analog_pins[i].last_value = value;
                // input_message[1] = the_analog_pins[i].pin_number;
                report_message[2] = (uint8_t) i;
                report_message[3] = value >> 8;
                report_message[4] = value & 0x00ff;
                serial_write(report_message, 5);
            }
        }
    }
}


/*************************************************
 * Write data to serial interface
 * @param buffer
 * @param num_of_bytes_to_send
 */
void serial_write(int *buffer, int num_of_bytes_to_send) {
    for (int i = 0; i < num_of_bytes_to_send; i++) {
        putchar((buffer[i]) & 0x00ff);
    }
    stdio_flush();

}

/***************************************************************
 *                  MAIN FUNCTION
 ****************************************************************/

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    stdio_flush();

    //stdio_set_translate_crlf(&stdio_usb, false);
    adc_init();
    // create an array of pin_descriptors for 100 pins
    // establish the digital pin array
    for (uint8_t i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        the_digital_pins[i].pin_number = i;
        the_digital_pins[i].pin_mode = PIN_MODE_NOT_SET;
        the_digital_pins[i].reporting_enabled = false;
        the_digital_pins[i].last_value = 0;
    }

    // establish the analog pin array
    for (uint8_t i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        the_analog_pins[i].reporting_enabled = false;
        the_analog_pins[i].last_value = 0;
    }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // blink the board LED twice to show that the board is
    // starting afresh
    led_debug(2, 250);

    // infinite loop
    while (true) {
        get_next_command();
        if (!stop_reports) {
            scan_digital_inputs();
            scan_analog_inputs();
            //scan_sonars();
            //scan_dhts();
        }


    }
}



