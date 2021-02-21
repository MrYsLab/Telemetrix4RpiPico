//
// Created by afy on 2/18/21.
//

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Differentiate between multiple Pico boards connected
#define PICO_ID 1


// We define these here to provide a forward reference.
// If you add a new command, you must add the command handler
// here as well.

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void pwm_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void are_you_there();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void serial_write(int *buffer, int num_of_bytes_to_send);

extern void led_debug(int blinks);

extern void send_debug_info(uint id, uint value);

extern void dht_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

void reset_data();


const uint LED_PIN = 25;

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define PWM_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define ARE_U_THERE  6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define DHT_NEW 14
#define STOP_ALL_REPORTS 15
#define SET_ANALOG_SCANNING_INTERVAL 16
#define ENABLE_ALL_REPORTS 17
#define RESET 18

#define MAX_DIGITAL_PINS_SUPPORTED 30
#define MAX_ANALOG_PINS_SUPPORTED 5

// Pin modes
#define DIGITAL_INPUT 0
#define DIGITAL_OUTPUT 1
#define PWM_OUTPUT 2
#define DIGITAL_INPUT_PULL_UP 3
#define DIGITAL_INPUT_PULL_DOWN 4
#define ANALOG_INPUT 5

#define PIN_MODE_NOT_SET 255

// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// Reports - sent from this sketch
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT 3
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define DEBUG_PRINT 99

// firmware version
#define FIRMWARE_MAJOR 0
#define FIRMWARE_MINOR 1

// a descriptor for digital pins
typedef struct {
    uint pin_number;
    uint pin_mode;
    uint reporting_enabled; // If true, then send reports if an input pin
    int last_value;        // Last value read for input mode
} pin_descriptor;

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// an array of analog_pin_descriptors
pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

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

// An array of pointers to the command functions
command_descriptor command_table[19] =
{
    {&serial_loopback},
    {&set_pin_mode},
    {&digital_write},
    {&pwm_write},
    {&modify_reporting},
    {&get_firmware_version},
    {&are_you_there},
    {&servo_attach},
    {&servo_write},
    {&servo_detach},
    {&i2c_begin},
    {&i2c_read},
    {&i2c_write},
    {&sonar_new},
    {&dht_new},
    {&stop_all_reports},
    {&set_analog_scanning_interval},
    {&enable_all_reports},
    {&reset_data}
};



// buffer to hold incoming command data
uint command_buffer[MAX_COMMAND_LENGTH];

void serial_loopback() {
    int loop_back_buffer[3] = {2, (int) SERIAL_LOOP_BACK, command_buffer[1]};
    serial_write(loop_back_buffer, 3);
}

void set_pin_mode() {
    uint pin;
    uint mode;
    uint16_t range;
    pwm_config pwm_cfg;
    pin = command_buffer[1];
    mode = command_buffer[2];

    switch (mode) {
        case DIGITAL_INPUT:
        case DIGITAL_INPUT_PULL_UP:
        case DIGITAL_INPUT_PULL_DOWN:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            break;
        case DIGITAL_OUTPUT:
            the_digital_pins[pin].pin_mode = mode;
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            break;
        case PWM_OUTPUT:
            range = (command_buffer[3] << 8) +  command_buffer[4];
            the_digital_pins[pin].pin_mode = mode;
            pwm_cfg = pwm_get_default_config();
            pwm_config_set_wrap(&pwm_cfg, range);
            pwm_init(pwm_gpio_to_slice_num(pin), &pwm_cfg, true);
            gpio_set_function(pin, GPIO_FUNC_PWM);
            break;

        case ANALOG_INPUT:
            the_analog_pins[pin].pin_mode = mode;
            the_analog_pins[pin].reporting_enabled = command_buffer[2];
            break;
        default:
            break;
    }
}

void digital_write() {
    uint pin;
    uint value;
    pin = command_buffer[1];
    value = command_buffer[2];
    gpio_put(pin, (bool)value);
}

void pwm_write(){
    uint pin;
    uint16_t value;

    pin = command_buffer[1];
    value = (command_buffer[2] >> 8) + command_buffer[3];
    pwm_set_gpio_level(pin, value);
}

void modify_reporting(){
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
            if (the_analog_pins[pin].pin_mode != PIN_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_ANALOG_DISABLE:
            if (the_analog_pins[pin].pin_mode != PIN_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = false;
            }
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

void get_firmware_version(){
    int report_message[4] = {3, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR};
    serial_write(report_message, 4);
}

void are_you_there() {
    int report_message[3] = {2, I_AM_HERE, PICO_ID};
    serial_write(report_message, 3);
}

void servo_attach() {}
void servo_write() {}
void servo_detach() {}
void i2c_begin() {}
void i2c_read() {}
void i2c_write() {}
void sonar_new() {}
void dht_new(){}
void stop_all_reports() {}
void set_analog_scanning_interval() {}
void enable_all_reports() {}
void reset_data() {}

void get_next_command() {
    int packet_size;
    int packet_data;
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
            if ((packet_data = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT) {
                sleep_ms(1);
            }
            command_buffer[i] = (uint)packet_data;
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

void serial_write(int *buffer, int num_of_bytes_to_send) {
    for (int i = 0; i < num_of_bytes_to_send; i++) {
        putchar((buffer[i]));
    }
}

void led_debug(int blinks) {
    for (int i = 0; i < blinks; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
}

// A method to send debug data across the serial link
void send_debug_info(uint id, uint value) {
    int debug_buffer[5] = { 4, DEBUG_PRINT, 0, 0, 0};
    debug_buffer[2] = id;
    debug_buffer[3] = (value & 0xff00) >> 8;
    debug_buffer[4] = value & 0x00ff;
    serial_write(debug_buffer, 5);
}

int main() {
    stdio_init_all();

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
        the_analog_pins[i].pin_number = i;
        the_analog_pins[i].pin_mode = PIN_MODE_NOT_SET;
        the_analog_pins[i].reporting_enabled = false;
        the_analog_pins[i].last_value = 0;
    }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        get_next_command();

    }
    return 0;

}



