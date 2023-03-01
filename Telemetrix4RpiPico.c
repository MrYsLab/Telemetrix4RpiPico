
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

/******************** Attributions ***********************************
 * This file contains modifications of the work of others to support some
 * of the project's features.
 *
 * Neopixel support: https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812
 *
 * DHT sensor support: https://github.com/raspberrypi/pico-examples/tree/master/gpio/dht_sensor
 *
 * HC-SR04 sensor support: https://github.com/GitJer/Some_RPI-Pico_stuff/tree/main/HCSR04
 *
 *************************************************************************/

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#include "include/Telemetrix4RpiPico.h"

/*******************************************************************
 *              GLOBAL VARIABLES, AND STORAGE
 ******************************************************************/

const uint LED_PIN = 25; // board LED

// buffer to hold incoming command data
uint8_t command_buffer[MAX_COMMAND_LENGTH];

bool stop_reports = false; // a flag to stop sending all report messages

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

// number of active sonars
int sonar_count = -1;
uint sonar_offset;

// hc-sr04 pio support values
PIO sonar_pio = pio1;

// sonar device descriptors
sonar_data the_hc_sr04s = {.next_sonar_index = 0};

// number of active dht devices
int dht_count = -1;

// dht device descriptors
dht_data the_dhts = {.next_dht_index = 0};

// pio for neopixel values
PIO np_pio = pio0;
uint np_sm = 0;

// neopixel storage for up to 150 pixel string
// Each entry contains an RGG array.

uint8_t pixel_buffer[MAXIMUM_NUM_NEOPIXELS][3];

uint actual_number_of_pixels;

// scan delay
uint8_t scan_delay = 0;

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);

}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// PWM values
uint32_t top;

// for dht repeating read timer
struct repeating_timer timer;
volatile bool timer_fired = false;


/******************* REPORT BUFFERS *******************/
// NOTE First value in the array is the number of reporting
// data elements. It does not include itself in this count.

// buffer to hold data for the loop_back command
// The last element will be filled in by the loopback command
int loop_back_report_message[] = {2, (int) SERIAL_LOOP_BACK, 0};

// buffer to hold data for send_debug_info command
uint debug_info_report_message[] = {4, DEBUG_PRINT, 0, 0, 0};

// buffer to hold firmware version info
int firmware_report_message[] = {3, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR};

// buffer to hold i2c report data
int i2c_report_message[64];

// buffer to hold spi report data
int spi_report_message[64];


// get_pico_unique_id report buffer
int unique_id_report_report_message[] = {9, REPORT_PICO_UNIQUE_ID,
                                         0, 0, 0, 0, 0, 0, 0, 0};
// digital input report buffer
int digital_input_report_message[] = {3, DIGITAL_REPORT, 0, 0};

// analog input report message
int analog_input_report_message[] = {4, ANALOG_REPORT, 0, 0, 0};

// sonar report message
int sonar_report_message[] = {4, SONAR_DISTANCE, 0, 0, 0};

// dht report message
int dht_report_message[] = {6, DHT_REPORT, 0, 0, 0, 0, 0,};

/*****************************************************************
 *                   THE COMMAND TABLE
 When adding a new command update the command_table.
 The command length is the number of bytes that follow
 the command byte itself, and does not include the command
 byte in its length.
 The command_func is a pointer the command's function.
 ****************************************************************/
// An array of pointers to the command functions
command_descriptor command_table[] =
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
                {&init_neo_pixels},
                {&show_neo_pixels},
                {&set_neo_pixel},
                {&clear_all_neo_pixels},
                {&fill_neo_pixels},
                {&init_spi},
                {&write_blocking_spi},
                {&read_blocking_spi},
                {&set_format_spi},
                {&spi_cs_control},
                {&set_scan_delay},
        };


/***************************************************************************
 *                   DEBUGGING FUNCTIONS
 **************************************************************************/

/************************************************************
 * Loop back the received character
 */
void serial_loopback() {
    loop_back_report_message[LOOP_BACK_DATA] = command_buffer[DATA_TO_LOOP_BACK];
    serial_write(loop_back_report_message,
                 sizeof(loop_back_report_message) / sizeof(int));
}

/******************************************************************
 * Send debug info report
 * @param id: 8 bit value
 * @param value: 16 bit value
 */
// A method to send debug data across the serial link
void send_debug_info(uint id, uint value) {
    debug_info_report_message[DEBUG_ID] = id;
    debug_info_report_message[DEBUG_VALUE_HIGH_BYTE] = (value & 0xff00) >> 8;
    debug_info_report_message[DEBUG_VALUE_LOW_BYTE] = value & 0x00ff;
    serial_write((int *) debug_info_report_message,
                 sizeof(debug_info_report_message) / sizeof(int));
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
    pin = command_buffer[SET_PIN_MODE_GPIO_PIN];
    mode = command_buffer[SET_PIN_MODE_MODE_TYPE];

    switch (mode) {
        case DIGITAL_INPUT:
        case DIGITAL_INPUT_PULL_UP:
        case DIGITAL_INPUT_PULL_DOWN:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[SET_PIN_MODE_DIGITAL_IN_REPORTING_STATE];
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
            /* Here we will set the operating frequency to be 50 hz to
               simplify support PWM as well as servo support.
            */
            the_digital_pins[pin].pin_mode = mode;

            const uint32_t f_hz = 50; // frequency in hz.

            uint slice_num = pwm_gpio_to_slice_num(pin); // get PWM slice for the pin

            // set frequency
            // determine top given Hz using the free-running clock
            uint32_t f_sys = clock_get_hz(clk_sys);
            float divider = (float) (f_sys / 1000000UL);  // run the pwm clock at 1MHz
            pwm_set_clkdiv(slice_num, divider); // pwm clock should now be running at 1MHz
            top = 1000000UL / f_hz - 1; // calculate the TOP value
            pwm_set_wrap(slice_num, (uint16_t) top);

            // set the current level to 0
            pwm_set_gpio_level(pin, 0);

            pwm_set_enabled(slice_num, true); // let's go!
            gpio_set_function(pin, GPIO_FUNC_PWM);
            break;

        case ANALOG_INPUT:
            //if the temp sensor was selected, then turn it on
            if (pin == ADC_TEMPERATURE_REGISTER) {
                adc_set_temp_sensor_enabled(true);
            }
            the_analog_pins[pin].reporting_enabled = command_buffer[SET_PIN_MODE_ANALOG_IN_REPORTING_STATE];
            // save the differential value
            the_analog_pins[pin].differential =
                    (int) ((command_buffer[SET_PIN_MODE_ANALOG_DIFF_HIGH] << 8) +
                           command_buffer[SET_PIN_MODE_ANALOG_DIFF_LOW]);
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
    pin = command_buffer[DIGITAL_WRITE_GPIO_PIN];
    value = command_buffer[DIGITAL_WRITE_VALUE];
    gpio_put(pin, (bool) value);
}

/**********************************************
 * Set A PWM Pin's value
 */
void pwm_write() {
    uint pin;
    uint16_t value;

    pin = command_buffer[PWM_WRITE_GPIO_PIN];

    value = (command_buffer[SET_PIN_MODE_PWM_HIGH_VALUE] << 8) +
            command_buffer[SET_PIN_MODE_PWM_LOW_VALUE];
    pwm_set_gpio_level(pin, value);
}

/***************************************************
 *  Control reporting
 */
void modify_reporting() {
    int pin = command_buffer[MODIFY_REPORTING_PIN];

    switch (command_buffer[MODIFY_REPORTING_TYPE]) {
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
    serial_write(firmware_report_message,
                 sizeof(firmware_report_message) / sizeof(int));
}

/**************************************************************
 * Retrieve the Pico's Unique ID
 */
void get_pico_unique_id() {
    // get the unique id
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    unique_id_report_report_message[2] = (board_id.id[0]);
    unique_id_report_report_message[3] = (board_id.id[1]);
    unique_id_report_report_message[4] = (board_id.id[2]);
    unique_id_report_report_message[5] = (board_id.id[3]);
    unique_id_report_report_message[6] = (board_id.id[4]);
    unique_id_report_report_message[7] = (board_id.id[5]);

    serial_write(unique_id_report_report_message,
                 sizeof(unique_id_report_report_message) / sizeof(int));
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

    // length of i2c report packet
    int num_of_bytes_to_send = I2C_READ_START_OF_DATA + command_buffer[I2C_READ_LENGTH];

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
            return;
        }
    }

    // now do the read request
    i2c_sdk_call_return_value = i2c_read_blocking(i2c,
                                                  (uint8_t) command_buffer[I2C_DEVICE_ADDRESS],
                                                  data_from_device,
                                                  (size_t) (command_buffer[I2C_READ_LENGTH]),
                                                  (bool) command_buffer[I2C_READ_NO_STOP_FLAG]);
    if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
        i2c_report_message[I2C_PACKET_LENGTH] = I2C_ERROR_REPORT_LENGTH; // length of the packet
        i2c_report_message[I2C_REPORT_ID] = I2C_READ_FAILED; //report ID
        i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];
        i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] = command_buffer[I2C_DEVICE_ADDRESS];

        serial_write(i2c_report_message, I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND);
        return;
    }

    // copy the data returned from i2c device into the report message buffer
    for (uint i = 0; i < i2c_sdk_call_return_value; i++) {
        i2c_report_message[i + I2C_READ_START_OF_DATA] = data_from_device[i];
    }
    // length of the packet
    i2c_report_message[I2C_PACKET_LENGTH] = (uint8_t) (i2c_sdk_call_return_value +
                                                       I2C_READ_DATA_BASE_BYTES);

    i2c_report_message[I2C_REPORT_ID] = I2C_READ_REPORT;

    // i2c_port
    i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];

    // i2c_address
    i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] = command_buffer[I2C_DEVICE_ADDRESS];

    // i2c register
    i2c_report_message[I2C_REPORT_READ_REGISTER] = command_buffer[I2C_READ_REGISTER];

    // number of bytes read from i2c device
    i2c_report_message[I2C_REPORT_READ_NUMBER_DATA_BYTES] = (uint8_t) i2c_sdk_call_return_value;

    serial_write((int *) i2c_report_message, num_of_bytes_to_send);

}

void i2c_write() {
    // i2c instance pointer
    i2c_inst_t *i2c;

    // Determine the i2c port to use.
    if (command_buffer[I2C_PORT]) {
        i2c = i2c1;
    } else {
        i2c = i2c0;
    }

    int i2c_sdk_call_return_value = i2c_write_blocking(i2c, (uint8_t) command_buffer[I2C_DEVICE_ADDRESS],
                                                       &(command_buffer[I2C_WRITE_BYTES_TO_WRITE]),
                                                       command_buffer[I2C_WRITE_NUMBER_OF_BYTES],
                                                       (bool) command_buffer[I2C_WRITE_NO_STOP_FLAG]);

    if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
        i2c_report_message[I2C_PACKET_LENGTH] = I2C_ERROR_REPORT_LENGTH; // length of the packet
        i2c_report_message[I2C_REPORT_ID] = I2C_WRITE_FAILED; //report ID
        i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];
        i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] = command_buffer[I2C_DEVICE_ADDRESS];

        serial_write(i2c_report_message, I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND);
        return;
    }
}

void init_neo_pixels() {
    // initialize the pico support a NeoPixel string
    uint offset = pio_add_program(np_pio, &ws2812_program);
    ws2812_init(np_pio, np_sm, offset, command_buffer[NP_PIN_NUMBER], 800000,
                false);

    actual_number_of_pixels = command_buffer[NP_NUMBER_OF_PIXELS];

    // set the pixels to the fill color
    for (int i = 0; i < actual_number_of_pixels; i++) {
        pixel_buffer[i][RED] = command_buffer[NP_RED_FILL];
        pixel_buffer[i][GREEN] = command_buffer[NP_GREEN_FILL];
        pixel_buffer[i][BLUE] = command_buffer[NP_BLUE_FILL];
    }
    show_neo_pixels();
    sleep_ms(1);

}

void set_neo_pixel() {
    // set a single neopixel in the pixel buffer
    pixel_buffer[command_buffer[NP_PIXEL_NUMBER]][RED] = command_buffer[NP_SET_RED];
    pixel_buffer[command_buffer[NP_PIXEL_NUMBER]][GREEN] = command_buffer[NP_SET_GREEN];
    pixel_buffer[command_buffer[NP_PIXEL_NUMBER]][BLUE] = command_buffer[NP_SET_BLUE];
    if (command_buffer[NP_SET_AUTO_SHOW]) {
        show_neo_pixels();
    }
}

void show_neo_pixels() {
    // show the neopixels in the buffer
    for (int i = 0; i < actual_number_of_pixels; i++) {
        put_pixel(urgb_u32(pixel_buffer[i][RED],
                           pixel_buffer[i][GREEN],
                           pixel_buffer[i][BLUE]));
    }
}

void clear_all_neo_pixels() {
    // set all the neopixels in the buffer to all zeroes
    for (int i = 0; i < actual_number_of_pixels; i++) {
        pixel_buffer[i][RED] = 0;
        pixel_buffer[i][GREEN] = 0;
        pixel_buffer[i][BLUE] = 0;
    }
    if (command_buffer[NP_CLEAR_AUTO_SHOW]) {
        show_neo_pixels();
    }
}

void fill_neo_pixels() {
    // fill all the neopixels in the buffer with the
    // specified rgb values.
    for (int i = 0; i < actual_number_of_pixels; i++) {
        pixel_buffer[i][RED] = command_buffer[NP_FILL_RED];
        pixel_buffer[i][GREEN] = command_buffer[NP_FILL_GREEN];
        pixel_buffer[i][BLUE] = command_buffer[NP_FILL_BLUE];
    }
    if (command_buffer[NP_FILL_AUTO_SHOW]) {
        show_neo_pixels();
    }
}

void sonar_new() {
    // add the sonar to the sonar struct to be processed within
    // the main loop
    uint trig_pin = command_buffer[SONAR_TRIGGER_PIN];
    uint echo_pin = command_buffer[SONAR_ECHO_PIN];

    // for the first HC-SR04, add the program.
    if (sonar_count == -1) {
        sonar_offset = pio_add_program(sonar_pio, &hc_sr04_program);
    }
    sonar_count++;
    if (sonar_count > MAX_SONARS) {
        return;
    }
    the_hc_sr04s.sonars[sonar_count].trig_pin = trig_pin;
    the_hc_sr04s.sonars[sonar_count].echo_pin = echo_pin;

    hc_sr04_init(sonar_pio, (uint) sonar_count, sonar_offset, trig_pin, echo_pin);
}

bool repeating_timer_callback(struct repeating_timer *t) {
    //printf("Repeat at %lld\n", time_us_64());
    timer_fired = true;
    return true;
}

void dht_new() {
    if (dht_count > MAX_DHTS) {
        return;
    }
    if(dht_count == -1){
        // first time through start repeating timer
        add_repeating_timer_ms(2000, repeating_timer_callback, NULL, &timer);
    }
    dht_count++;

    uint dht_pin = command_buffer[DHT_DATA_PIN];
    the_dhts.dhts[dht_count].data_pin = dht_pin;
    the_dhts.dhts[dht_count].previous_time = get_absolute_time();
    gpio_init(dht_pin);
}

void init_spi(){
    spi_inst_t *spi_port;
    uint spi_baud_rate;
    uint cs_pin;

    // initialize the spi port
    if(command_buffer[SPI_PORT] == 0){
        spi_port = spi0;
    }
    else{
        spi_port = spi1;
    }

    spi_baud_rate = ((command_buffer[SPI_FREQ_MSB] << 24) +
            (command_buffer[SPI_FREQ_3] << 16) +
            (command_buffer[SPI_FREQ_2] << 8) +
            (command_buffer[SPI_FREQ_1] ));

    spi_init(spi_port, spi_baud_rate);

    // set gpio pins for miso, mosi and clock
    gpio_set_function(command_buffer[SPI_MISO], GPIO_FUNC_SPI);
    gpio_set_function(command_buffer[SPI_MOSI], GPIO_FUNC_SPI);
    gpio_set_function(command_buffer[SPI_CLK_PIN], GPIO_FUNC_SPI);

    // initialize chip select GPIO pins
    for(int i = 0; i < command_buffer[SPI_CS_LIST_LENGTH]; i++){
        cs_pin = command_buffer[SPI_CS_LIST + i];
        // Chip select is active-low, so we'll initialise it to a driven-high state
        gpio_init(cs_pin);
        gpio_set_dir(cs_pin, GPIO_OUT);
        gpio_put(cs_pin, 1);
    }
}

void spi_cs_control(){
    uint8_t cs_pin;
    uint8_t cs_state;

    cs_pin = command_buffer[SPI_SELECT_PIN];
    cs_state = command_buffer[SPI_SELECT_STATE];
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, cs_state);
    asm volatile("nop \n nop \n nop");
}

void set_scan_delay(){

    scan_delay = command_buffer[SCAN_DELAY];
}

void read_blocking_spi(){
    // The report_message offsets:
    // 0 = packet length - this must be calculated
    // 1 = SPI_READ_REPORT
    // 2 = The i2c port - 0 or 1
    // 3 = number of bytes read
    // 4... = bytes read


    spi_inst_t *spi_port;
    size_t data_length;
    uint8_t repeated_transmit_byte;
    uint8_t data[command_buffer[SPI_READ_LEN]];

    if(command_buffer[SPI_PORT] == 0){
        spi_port = spi0;
    }
    else{
        spi_port = spi1;
    }

    data_length = command_buffer[SPI_READ_LEN];
    //memset(data, 0, data_length);
    memset(data, 0, sizeof(data));

    repeated_transmit_byte = command_buffer[SPI_REPEATED_DATA];

    // read data
    spi_read_blocking(spi_port, repeated_transmit_byte, data, data_length);
    sleep_ms(100);

    // build a report from the data returned
    spi_report_message[SPI_PACKET_LENGTH] = SPI_REPORT_NUMBER_OF_DATA_BYTES + data_length;
    spi_report_message[SPI_REPORT_ID] = SPI_REPORT;
    spi_report_message[SPI_REPORT_PORT] = command_buffer[SPI_PORT];
    spi_report_message[SPI_REPORT_NUMBER_OF_DATA_BYTES] = data_length;
    for(int i=0; i < data_length; i++){
        spi_report_message[SPI_DATA + i] = data[i];
    }
    serial_write((int *) spi_report_message,
                 SPI_DATA + data_length);

}

void write_blocking_spi() {
    spi_inst_t *spi_port;
    uint cs_pin;
    size_t data_length;

    if(command_buffer[SPI_PORT] == 0){
        spi_port = spi0;
    }
    else{
        spi_port = spi1;
    }

    data_length = command_buffer[SPI_WRITE_LEN];
    // write data
    spi_write_blocking(spi_port, &command_buffer[SPI_WRITE_DATA], data_length);
}


void set_format_spi(){
    spi_inst_t *spi_port;
    uint data_bits = command_buffer[SPI_NUMBER_OF_BITS];
    spi_cpol_t cpol = command_buffer[SPI_CLOCK_PHASE];
    spi_cpha_t cpha = command_buffer[SPI_CLOCK_POLARITY];

    if(command_buffer[SPI_PORT] == 0){
        spi_port = spi0;
    }
    else{
        spi_port = spi1;
    }
    spi_set_format(spi_port, data_bits, cpol, cpha, 1);
}


/******************* FOR FUTURE RELEASES **********************/

void reset_data() {}

/***************** Currently Unused ***************************/
void servo_attach() {}

void servo_write() {}

void servo_detach() {}

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

    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        if (the_digital_pins[i].pin_mode == DIGITAL_INPUT ||
            the_digital_pins[i].pin_mode == DIGITAL_INPUT_PULL_UP ||
            the_digital_pins[i].pin_mode == DIGITAL_INPUT_PULL_DOWN) {
            if (the_digital_pins[i].reporting_enabled) {
                // if the value changed since last read
                value = gpio_get(the_digital_pins[i].pin_number);
                if (value != the_digital_pins[i].last_value) {
                    the_digital_pins[i].last_value = value;
                    digital_input_report_message[DIGITAL_INPUT_GPIO_PIN] = i;
                    digital_input_report_message[DIGITAL_INPUT_GPIO_VALUE] = value;
                    serial_write(digital_input_report_message, 4);
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
                analog_input_report_message[ANALOG_INPUT_GPIO_PIN] = (uint8_t) i;
                analog_input_report_message[ANALOG_VALUE_HIGH_BYTE] = value >> 8;
                analog_input_report_message[ANALOG_VALUE_LOW_BYTE] = value & 0x00ff;
                serial_write(analog_input_report_message, 5);
            }
        }
    }
}

void scan_sonars() {
    // read the next sonar device
    // one device is read each cycle
    if (sonar_count >= 0) {
        read_sonar(the_hc_sr04s.next_sonar_index);
        the_hc_sr04s.next_sonar_index++;
        if (the_hc_sr04s.next_sonar_index > sonar_count) {
            the_hc_sr04s.next_sonar_index = 0;
        }
    }
}

void read_sonar(uint sm) {
    // value is used to read from the sm RX FIFO
    uint32_t clock_cycles;
    // clear the FIFO: do a new measurement
    pio_sm_clear_fifos(sonar_pio, sm);
    // give the sm some time to do a measurement and place it in the FIFO
    sleep_ms(100);
    // check that the FIFO isn't empty
    if (pio_sm_is_rx_fifo_empty(sonar_pio, sm)) {
        // its empty so create a report returning a distance of zero
        sonar_report_message[SONAR_TRIG_PIN] = (uint8_t) the_hc_sr04s.sonars[sm].trig_pin;
        sonar_report_message[CM_WHOLE_VALUE] = 0;
        sonar_report_message[CM_FRAC_VALUE] = 0;
        serial_write(sonar_report_message, 5);
        return;
    }

    // read one data item from the FIFO
    // Note: every test for the end of the echo pulse takes 2 pio clock ticks,
    //       but changes the 'timer' by only one
    clock_cycles = 2 * pio_sm_get(sonar_pio, sm);
    // using
    // - the time for 1 pio clock tick (1/125000000 s)
    // - speed of sound in air is about 340 m/s
    // - the sound travels from the HCS-R04 to the object and back (twice the distance)
    // we can calculate the distance in cm by multiplying with 0.000136
    float cm = (float) clock_cycles * 0.000136;

    // convert the value into 2 integers - left and right of the decimal point
    float nearest = roundf(cm * 100) / 100;
    int intpart = (int) nearest;
    int decpart = (int) ((nearest - intpart) * 100);

    sonar_report_message[SONAR_TRIG_PIN] = (uint8_t) the_hc_sr04s.sonars[sm].trig_pin;
    sonar_report_message[CM_WHOLE_VALUE] = intpart;
    sonar_report_message[CM_FRAC_VALUE] = decpart;
    serial_write(sonar_report_message, 5);
}

void scan_dhts() {
    // read the next dht device
    // one device is read each cycle
    if (dht_count >= 0) {
        if (timer_fired) {
            timer_fired = false;
            int the_index = the_dhts.next_dht_index;
            read_dht(the_dhts.dhts[the_index].data_pin);
            the_dhts.next_dht_index++;
            if (the_dhts.next_dht_index > dht_count) {
                the_dhts.next_dht_index = 0;
            }
        }
    }
}

void read_dht(uint dht_pin) {
    int data[5] = {0, 0, 0, 0, 0};
    uint last = 1;
    uint j = 0;
    float temp_celsius;
    float humidity;
    float nearest;
    int temp_int_part;
    int temp_dec_part;
    int humidity_int_part;
    int humidity_dec_part;

    gpio_set_dir(dht_pin, GPIO_OUT);
    gpio_put(dht_pin, 0);
    sleep_ms(20);
    gpio_set_dir(dht_pin, GPIO_IN);

    sleep_us(1);
    for (uint i = 0; i < DHT_MAX_TIMINGS; i++) {
        uint count = 0;
        while (gpio_get(dht_pin) == last) {
            count++;
            sleep_us(1);
            if (count == 255) break;
        }
        last = gpio_get(dht_pin);
        if (count == 255) break;

        if ((i >= 4) && (i % 2 == 0)) {
            data[j / 8] <<= 1;
            if (count > 46) data[j / 8] |= 1;
            j++;
        }
    }
    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        humidity = (float) ((data[0] << 8) + data[1]) / 10;
        if (humidity > 100) {
            humidity = data[0];
        }
        temp_celsius = (float) (((data[2] & 0x7F) << 8) + data[3]) / 10;
        if (temp_celsius > 125) {
            temp_celsius = data[2];
        }
        if (data[2] & 0x80) {
            temp_celsius = -temp_celsius;
        }

        nearest = roundf(temp_celsius * 100) / 100;
        temp_int_part = (int) nearest;
        temp_dec_part = (int) ((nearest - temp_int_part) * 100);

        nearest = roundf(humidity * 100) / 100;
        humidity_int_part = (int) nearest;
        humidity_dec_part = (int) ((nearest - humidity_int_part) * 100);


    } else {
        // bad data return zeros
        temp_int_part = temp_dec_part =
        humidity_int_part = humidity_dec_part = 0;
    }
    dht_report_message[DHT_REPORT_PIN] = (int) dht_pin;
    dht_report_message[DHT_HUMIDITY_WHOLE_VALUE] = humidity_int_part;
    dht_report_message[DHT_HUMIDITY_FRAC_VALUE] = humidity_dec_part;

    dht_report_message[DHT_TEMPERATURE_WHOLE_VALUE] = temp_int_part;
    dht_report_message[DHT_TEMPERATURE_FRAC_VALUE] = temp_dec_part;
    serial_write(dht_report_message, 7);
}


/*************************************************
 * Write data to serial interface
 * @param buffer
 * @param num_of_bytes_to_send
 */
void serial_write(const int *buffer, int num_of_bytes_to_send) {
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
    //uint offset = pio_add_program(pio, &Telemetrix4RpiPico_program);
    //ws2812_init(pio, sm, offset, 28, 800000,
    //            false);

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

    // initialize the sonar structures
    sonar_data the_hc_sr04s = {.next_sonar_index = 0};
    for (int i = 0; i < MAX_SONARS; i++) {
        the_hc_sr04s.sonars[i].trig_pin = the_hc_sr04s.sonars[i].echo_pin = (uint) -1;
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
            scan_sonars();
            scan_dhts();
            sleep_ms(scan_delay);
        }


    }
}


#pragma clang diagnostic pop