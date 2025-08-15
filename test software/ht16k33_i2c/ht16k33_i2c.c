/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <ctype.h>

/* Example code to drive a 4 digit 14 segment LED backpack using a HT16K33 I2C
   driver chip

   NOTE: The panel must be capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore I2C) cannot be used at 5v. In development the particular 
   device used allowed the PCB VCC to be 5v, but you can set the I2C voltage 
   to 3.3v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO 4 (pin 6)-> SDA on LED board
   GPIO 5 (pin 7)-> SCL on LED board
   GND (pin 38)  -> GND on LED board
   5v (pin 40)   -> VCC on LED board
   3.3v (pin 36) -> vi2c on LED board
*/

// How many digits are on our display.
#define NUM_DIGITS 4

// By default these display drivers are on bus address 0x70. Often there are
// solder on options on the PCB of the backpack to set an address between
// 0x70 and 0x77 to allow multiple devices to be used.
const int I2C_addr = 0x70;
#define HT16K33_ADDRESS 0x70

#ifndef I2C_PORT
#define I2C_PORT i2c0
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 20
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 21
#endif

// commands
#define HT16K33_SYSTEM_STANDBY  0x20
#define HT16K33_SYSTEM_RUN      0x21

#define HT16K33_SET_ROW_INT     0xA0

#define HT16K33_BRIGHTNESS      0xE0

// Display on/off/blink
#define HT16K33_DISPLAY_SETUP   0x80
// OR/clear these to display setup register
#define HT16K33_DISPLAY_OFF     0x0
#define HT16K33_DISPLAY_ON      0x1
#define HT16K33_BLINK_2HZ       0x2
#define HT16K33_BLINK_1HZ       0x4
#define HT16K33_BLINK_0p5HZ     0x6

#define a 1
#define b 2
#define c 4
#define d 8
#define e 16
#define f 32
#define g 64
#define DP 128

// 74hc595 spi defines
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SHCP  34
#define PIN_DS 35
#define PIN_STCP 36
#define PIN_OE 37
#define PIN_MR 38

// MMA8451Q Accelerometer commands
#define MMA8451Q_ADDRESS 0x1C

// MMA8451Q Accelerometer register addresses
#define MMA8451Q_F_STATUS 0x00
#define MMA8451Q_OUT_X_MSB 0x01 
#define MMA8451Q_OUT_X_LSB 0x02
#define MMA8451Q_OUT_Y_MSB 0x03
#define MMA8451Q_OUT_Y_LSB 0x04
#define MMA8451Q_OUT_Z_MSB 0x05
#define MMA8451Q_OUT_Z_LSB 0x06
#define MMA8451Q_F_SETUP 0x09
#define MMA8451Q_TRIG_CFG 0x0A
#define MMA8451Q_SYSMOD 0x0B
#define MMA8451Q_INT_SOURCE 0x0C
#define MMA8451Q_WHO_AM_I 0x0D
#define MMA8451Q_XYZ_DATA_CFG 0x0E
#define MMA8451Q_HP_FILTER_CUTOFF 0x0F
#define MMA8451Q_PL_STATUS 0x10
#define MMA8451Q_PL_CFG 0x11
#define MMA8451Q_PL_COUNT 0x12
#define MMA8451Q_PL_BF_ZCOMP 0x13
#define MMA8451Q_P_L_THS_REG 0x14
#define MMA8451Q_FF_MT_CFG 0x15
#define MMA8451Q_FF_MT_SRC 0x16
#define MMA8451Q_FF_MT_THS 0x17
#define MMA8451Q_FF_MT_COUNT 0x18
#define MMA8451Q_TRANSIENT_CFG 0x1D
#define MMA8451Q_TRANSIENT_SRC 0x1E
#define MMA8451Q_TRANSIENT_THS 0x1F
#define MMA8451Q_TRANSIENT_COUNT 0x20
#define MMA8451Q_PULSE_CFG 0x21
#define MMA8451Q_PULSE_SRC 0x22
#define MMA8451Q_PULSE_THSX 0x23
#define MMA8451Q_PULSE_THSY 0x24
#define MMA8451Q_PULSE_THSZ 0x25
#define MMA8451Q_PULSE_TMLT 0x26
#define MMA8451Q_PULSE_LTCY 0x27
#define MMA8451Q_PULSE_WIND 0x28
#define MMA8451Q_ALSP_COUNT 0x29
#define MMA8451Q_CTRL_REG1 0x2A
#define MMA8451Q_CTRL_REG2 0x2B
#define MMA8451Q_CTRL_REG3 0x2C
#define MMA8451Q_CTRL_REG4 0x2D
#define MMA8451Q_CTRL_REG5 0x2E
#define MMA8451Q_OFF_X 0x2F
#define MMA8451Q_OFF_Y 0x30
#define MMA8451Q_OFF_Z 0x31

#define ACCEL_RANGE_2G      0b00
#define ACCEL_RANGE_4G      0b01
#define ACCEL_RANGE_8G      0b10

#define CURRENT_ACCEL_RANGE ACCEL_RANGE_2G

#define SENSITIVITY_2G      8192.0f
#define SENSITIVITY_4G      4096.0f
#define SENSITIVITY_8G      2048.0f

#define ODR_100HZ           0b011  // 100 Hz
#define CURRENT_ODR         ODR_100HZ

// button and switch define
#define BTN1 30
#define BTN2 31
#define BTN3 32
#define BTN4 33

#define SW1 22
#define SW2 23
#define SW3 24
#define SW4 25
#define SW5 26
#define SW6 27
#define SW7 28
#define SW8 29

#define LED_RED 17
#define LED_YELLOW 16
#define LED_GREEN 15

void ht16k33_clear_all(void);
bool mma8451q_init(void);

// Converts a character to the bit pattern needed to display the right segments.
// These are pretty standard for 14segment LED's
uint16_t char_to_pattern(char ch) {
    // Map, "A" to "Z"
    int16_t alpha[] = {
    0xF7,0x128F,0x39,0x120F,0xF9,0xF1,0xBD,0xF6,0x1209,0x1E,0x2470,0x38,0x536,0x2136,
    0x3F,0xF3,0x203F,0x20F3,0x18D,0x1201,0x3E,0xC30,0x2836,0x2D00,0x1500,0xC09
    };

    // Map, "0" to "9"
    //int16_t num[] = {0xC3F,0x406,0xDB,0x8F,0xE6,0xED,0xFD,0x1401,0xFF,0xE7};
    int16_t num[] = {0x3F, 0x06, 0x5B, 0X4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

    if (isalpha(ch))
        return alpha[toupper(ch) - 'A'];
    
    if (isdigit(ch))
        return num[ch - '0'];
    
    return 0;
}

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val, uint8_t address) {
    i2c_write_blocking(I2C_PORT, address, &val, 1, false);
}


void ht16k33_init() {
    i2c_write_byte(HT16K33_SYSTEM_RUN, HT16K33_ADDRESS);
    i2c_write_byte(HT16K33_SET_ROW_INT, HT16K33_ADDRESS);
    i2c_write_byte(HT16K33_DISPLAY_SETUP | HT16K33_DISPLAY_ON, HT16K33_ADDRESS);
    ht16k33_clear_all();
}

// Send a specific binary value to the specified digit
static inline void ht16k33_display_set(int position, uint16_t bin) {
    uint8_t buf[3];
    buf[0] = position * 2;
    buf[1] = bin & 0xff;
    buf[2] = bin >> 8;
    i2c_write_blocking(I2C_PORT, HT16K33_ADDRESS, buf, count_of(buf), false);

}

static inline void ht16k33_display_char(int position, char ch) {
    ht16k33_display_set(position, char_to_pattern(ch));
}
    
void ht16k33_display_string(char *str) {
    int digit = 0;
    while (*str && digit <= NUM_DIGITS) {
        ht16k33_display_char(digit++, *str++);
    }
}

void ht16k33_scroll_string(char *str, int interval_ms) {
    int l = strlen(str);

    if (l <= NUM_DIGITS) {
        ht16k33_display_string(str);
    }
    else {
        for (int i = 0; i < l - NUM_DIGITS + 1; i++) {
            ht16k33_display_string(&str[i]);
            sleep_ms(interval_ms);
        }
    }
}

void ht16k33_set_brightness(int bright) {
    i2c_write_byte(HT16K33_BRIGHTNESS | (bright <= 15 ? bright : 15), HT16K33_ADDRESS);
}

void ht16k33_set_blink(int blink) {
    int s = 0;
    switch (blink) {
        default: break;
        case 1: s = HT16K33_BLINK_2HZ; break;
        case 2: s = HT16K33_BLINK_1HZ; break;
        case 3: s = HT16K33_BLINK_0p5HZ; break;
    }

    i2c_write_byte(HT16K33_DISPLAY_SETUP | HT16K33_DISPLAY_ON | s, HT16K33_ADDRESS);
}

void ht16k33_clear_all() {
    ht16k33_display_set(0, 0);
    ht16k33_display_set(1, 0);
    ht16k33_display_set(2, 0);
    ht16k33_display_set(3, 0);
    return;
}

void display_snake(int ms) {
    ht16k33_clear_all();
    ht16k33_display_set(0, a);
    sleep_ms(ms);
    ht16k33_display_set(1, a);
    ht16k33_display_set(0, 0);
    sleep_ms(ms);
    ht16k33_display_set(2, a);
    ht16k33_display_set(1, 0);
    sleep_ms(ms);
    ht16k33_display_set(3, a);
    ht16k33_display_set(2, 0);
    sleep_ms(ms);
    ht16k33_display_set(3, b);
    sleep_ms(ms);
    ht16k33_display_set(3, g);
    sleep_ms(ms);
    ht16k33_display_set(2, g);
    ht16k33_display_set(3, 0);
    sleep_ms(ms);
    ht16k33_display_set(1, g);
    ht16k33_display_set(2, 0);
    sleep_ms(ms);
    ht16k33_display_set(0, g);
    ht16k33_display_set(1, 0);
    sleep_ms(ms);
    ht16k33_display_set(0, e);
    sleep_ms(ms);
    ht16k33_display_set(0, d);
    sleep_ms(ms);
    ht16k33_display_set(1, d);
    ht16k33_display_set(0, 0);
    sleep_ms(ms);
    ht16k33_display_set(2, d);
    ht16k33_display_set(1, 0);
    sleep_ms(ms);
    ht16k33_display_set(3, d);
    ht16k33_display_set(2, 0);
    sleep_ms(ms);
    ht16k33_display_set(3, c);
    sleep_ms(ms);
    ht16k33_display_set(3, g);
    sleep_ms(ms);
    ht16k33_display_set(2, g);
    ht16k33_display_set(3, 0);
    sleep_ms(ms);
    ht16k33_display_set(1, g);
    ht16k33_display_set(2, 0);
    sleep_ms(ms);
    ht16k33_display_set(0, g);
    ht16k33_display_set(1, 0);
    sleep_ms(ms);
    ht16k33_display_set(0, f);
    sleep_ms(ms);
    ht16k33_clear_all();
    return;
}



int mma8451q_write_register(uint8_t reg_address, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg_address;
    buf[1] = value;
    
    int ret = i2c_write_blocking(I2C_PORT, MMA8451Q_ADDRESS, buf, 2, false);
    
    if (ret != 2) {
        printf("I2C Write Error to 0x%02X, ret: %d\n", reg_address, ret);
        return PICO_ERROR_GENERIC;
    }
    return ret;
}

int mma8451q_read_register(uint8_t reg_address, uint8_t *buffer, size_t len) {
    // first send (device address + write)
    // then send register address
    // first tell accelerometer which address to read from
    int ret = i2c_write_blocking(I2C_PORT, MMA8451Q_ADDRESS, &reg_address, 1, true);
    if (ret != 1) {
        printf("Accelerometer I2C read data error (write reg address)\n");
        return PICO_ERROR_GENERIC;
    }
    // then read from accelerometer
    ret = i2c_read_blocking(I2C_PORT, MMA8451Q_ADDRESS, buffer, len, false); // false stop bit
    if (ret != len) { // check if number of returned bytes is correct
        printf("Accelerometer I2C read data error (read data)\n");
        return PICO_ERROR_GENERIC;
    }
    return ret;
}



/**
 * @brief Converts a two's complement value to a signed integer.
 * @param val The 16-bit value to convert.
 * @param bits The number of bits representing the value (e.g., 14 for MMA8451Q).
 * @return The signed integer.
 */
int16_t twos_comp_to_int16(uint16_t val, uint8_t bits) {
    // If the most significant bit (MSB) of the actual data is set, it's a negative number
    if (val & (1 << (bits - 1))) {
        // Perform sign extension for a 16-bit signed integer
        return (int16_t)(val | (~((1 << bits) - 1)));
    }
    return (int16_t)val;
}

bool mma8451q_read_data(uint16_t* x, uint16_t* y, uint16_t* z) {
    uint8_t raw_data[6]; // X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB

    // read 6 bytes starting from OUT_X_MSB (0x01)
    if (mma8451q_read_register(MMA8451Q_OUT_X_MSB, raw_data, 6) == PICO_ERROR_GENERIC) {return false;}

    uint16_t raw_x_16bit = (raw_data[0] << 8) | raw_data[1];
    uint16_t raw_y_16bit = (raw_data[2] << 8) | raw_data[3];
    uint16_t raw_z_16bit = (raw_data[4] << 8) | raw_data[5];


    // convert 16 bit raw data to 14 bit signed integer
    // right shift by two to get 14 bit value, then convert to twos complement
    *x = twos_comp_to_int16(raw_x_16bit >> 2, 14);
    *y = twos_comp_to_int16(raw_y_16bit >> 2, 14);
    *z = twos_comp_to_int16(raw_z_16bit >> 2, 14);
 
    return true;
}

void trigger_74hc595_stcp() {
    gpio_put(PIN_STCP, true);
    gpio_put(PIN_STCP, false);
}

bool mma8451q_init() {
   printf("Initializing MMA8451Q...\n");
    uint8_t data_buffer[1];

    // 1. Check WHO_AM_I register
    // should return 0x1A according to datasheet, but tests show that it returns 0x2A
    if (mma8451q_read_register(MMA8451Q_WHO_AM_I, &data_buffer[0], 1) == PICO_ERROR_GENERIC) {
        printf("Error: Could not read WHO_AM_I register.\n");
        return false;
    }
    if (data_buffer[0] != 0x2A) {
        printf("Error: MMA8451Q not found or WHO_AM_I mismatch (expected 0x1A, got 0x%02X)\n", data_buffer[0]);
        return false;
    }
    printf("MMA8451Q found! WHO_AM_I: 0x%02X\n", data_buffer[0]);

    // set to standby mode
    if (mma8451q_read_register(MMA8451Q_CTRL_REG1, data_buffer, 1) == PICO_ERROR_GENERIC) return false;

    // clear active bit 0 to enter standby
    if (mma8451q_write_register(MMA8451Q_CTRL_REG1, data_buffer[0] & ~0x01) == PICO_ERROR_GENERIC) return false;
    sleep_ms(10); // Small delay after mode change

    // configure g-range
    if (mma8451q_read_register(MMA8451Q_XYZ_DATA_CFG, data_buffer, 1) == PICO_ERROR_GENERIC) return false;

    // clear bits 1-0 and then set new range
    uint8_t new_xyz_cfg = (data_buffer[0] & ~0x03) | CURRENT_ACCEL_RANGE;
    if (mma8451q_write_register(MMA8451Q_XYZ_DATA_CFG, new_xyz_cfg) == PICO_ERROR_GENERIC) return false;
    printf("Set accelerometer range to %dg\n", (1 << (CURRENT_ACCEL_RANGE + 1)));

    // set output data rate ODR and activate CTRL REG1
    if (mma8451q_read_register(MMA8451Q_CTRL_REG1, data_buffer, 1) == PICO_ERROR_GENERIC) return false;

    // clear ODR bits 2-0 and set new ODR, then set activate bit
    uint8_t new_ctrl_reg1 = (data_buffer[0] & ~0x07) | CURRENT_ODR | 0x01; // Set ACTIVE bit (0x01)
    if (mma8451q_write_register(MMA8451Q_CTRL_REG1, new_ctrl_reg1) == PICO_ERROR_GENERIC) return false;
    printf("MMA8451Q activated.\n");
    return true;

}

/**
 * @brief Converts a raw 14-bit acceleration value to g-force.
 * @param raw_value The raw 14-bit signed acceleration value.
 * @return The acceleration in g's.
 */
float convert_to_g(int16_t raw_value) {
    float sensitivity = 0.0f;
    if (CURRENT_ACCEL_RANGE == ACCEL_RANGE_2G) {
        sensitivity = SENSITIVITY_2G;
    } else if (CURRENT_ACCEL_RANGE == ACCEL_RANGE_4G) {
        sensitivity = SENSITIVITY_4G;
    } else if (CURRENT_ACCEL_RANGE == ACCEL_RANGE_8G) {
        sensitivity = SENSITIVITY_8G;
    } else {
        printf("Warning: Unknown accelerometer range selected!\n");
        return 0.0f; // Return 0 or handle error appropriately
    }

    if (sensitivity == 0.0f) {
        return 0.0f;
    }

    return (float)raw_value / sensitivity;
}

int main() {

    stdio_init_all();

    // traffic light leds
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_put(LED_GREEN, 0);

    gpio_init(LED_YELLOW);
    gpio_set_dir(LED_YELLOW, GPIO_OUT);
    gpio_put(LED_YELLOW, 0);

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, 0);
    
    

    // init pushbutton gpio
    gpio_init(BTN1);
    gpio_set_dir(BTN1, GPIO_IN);
    gpio_pull_up(BTN1);

    gpio_init(BTN2);
    gpio_set_dir(BTN2, GPIO_IN);
    gpio_pull_up(BTN2);

    gpio_init(BTN3);
    gpio_set_dir(BTN3, GPIO_IN);
    gpio_pull_up(BTN3);

    gpio_init(BTN4);
    gpio_set_dir(BTN4, GPIO_IN);
    gpio_pull_up(BTN4);
    

    // init slide switch gpio
    gpio_init(SW1);
    gpio_init(SW2);
    gpio_init(SW3);
    gpio_init(SW4);
    gpio_init(SW5);
    gpio_init(SW6);
    gpio_init(SW7);
    gpio_init(SW8);
    gpio_set_dir(SW1, GPIO_IN);
    gpio_set_dir(SW2, GPIO_IN);
    gpio_set_dir(SW3, GPIO_IN);
    gpio_set_dir(SW4, GPIO_IN);
    gpio_set_dir(SW5, GPIO_IN);
    gpio_set_dir(SW6, GPIO_IN);
    gpio_set_dir(SW7, GPIO_IN);
    gpio_set_dir(SW8, GPIO_IN);
    gpio_pull_up(SW1);
    gpio_pull_up(SW2);
    gpio_pull_up(SW3);
    gpio_pull_up(SW4);
    gpio_pull_up(SW5);
    gpio_pull_up(SW6);
    gpio_pull_up(SW7);
    gpio_pull_up(SW8);

/*#if !defined(I2C_PORT) || !defined(I2C_SDA_PIN) || !defined(I2C_SCL_PIN)
    #warning i2c/ht16k33_i2c example requires a board with I2C pins
#endif*/
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    // init 74hc595 SPI
    spi_init(spi0, 400*1000);
    gpio_init(PIN_MISO);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_DS);
    gpio_set_function(PIN_DS, GPIO_FUNC_SPI);
    gpio_init(PIN_SHCP);
    gpio_set_function(PIN_SHCP,  GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    
    gpio_init(PIN_STCP);
    gpio_set_dir(PIN_STCP, GPIO_OUT);
    gpio_put(PIN_STCP, false);

    gpio_init(PIN_OE);
    gpio_set_dir(PIN_OE, GPIO_OUT);
    gpio_put(PIN_OE, false);

    gpio_init(PIN_MR);
    gpio_set_dir(PIN_MR, GPIO_OUT);
    gpio_put(PIN_MR, true);


    // init ht16k33 after i2c init
    ht16k33_init();

    if (!mma8451q_init()) {
        printf("Failed to initialize MMA8451Q. Program will not read data.\n");
        while (true) { // Loop indefinitely on error
            sleep_ms(1000);
        }
    }

    // Test brightness and blinking
    // Set all segments on all digits on
    ht16k33_display_set(0, 0xff);
    ht16k33_display_set(1, 0xff);
    ht16k33_display_set(2, 0xff);
    ht16k33_display_set(3, 0xff);
    ht16k33_set_brightness(15);
    

    uint8_t spidata[3];
    spidata[0] = 0;
    spidata[1] = 0;
    spidata[2] = 0;

    int btn1, btn2, btn3, btn4;
    int sw1, sw2, sw3, sw4, sw5, sw6, sw7, sw8;

    int16_t raw_x, raw_y, raw_z;
    float g_x, g_y, g_z;

    // start program loop
    // pull-up inverts gpio read, so 'off' switch is read as 1
    while (true)
    {
        // buttons are wired to internal pull-up resistors, which means that a button not pressed will return 1, and pressed returns 0
        btn1 = gpio_get(BTN1);
        btn2 = gpio_get(BTN2);
        btn3 = gpio_get(BTN3);
        btn4 = gpio_get(BTN4);
         
        printf("button1: %d, button2: %d, button3: %d, button4: %d\n", btn1, btn2, btn3, btn4);

        // switches are also wired to internal pull-up resistors, which means that on = 0, off = 1, so we invert the value
        sw1 = !gpio_get(SW1);
        sw2 = !gpio_get(SW2);
        sw3 = !gpio_get(SW3);
        sw4 = !gpio_get(SW4);
        sw5 = !gpio_get(SW5);
        sw6 = !gpio_get(SW6);
        sw7 = !gpio_get(SW7);
        sw8 = !gpio_get(SW8);

        
        if (btn1 | btn2 | btn3 | btn4) {
            gpio_put(LED_RED, 1);
            gpio_put(LED_YELLOW, 1);
            //gpio_put(LED_GREEN, 1);
        } else {
            gpio_put(LED_RED, 0);
            gpio_put(LED_YELLOW, 0);
            gpio_put(LED_GREEN, 0);
        }

        // send switch data to shift register leds
        spidata[0] = (sw8<<7)+(sw7<<6)+(sw6<<5)+(sw5<<4)+(sw4<<3)+(sw3<<2)+(sw2<<1)+(sw1<<0);
        spidata[1] = (sw8<<7)+(sw7<<6)+(sw6<<5)+(sw5<<4)+(sw4<<3)+(sw3<<2)+(sw2<<1)+(sw1<<0);
        spidata[2] = (sw8<<7)+(sw7<<6)+(sw6<<5)+(sw5<<4)+(sw4<<3)+(sw3<<2)+(sw2<<1)+(sw1<<0);
    
        spi_write_blocking(spi0, &spidata[2], 1);
        spi_write_blocking(spi0, &spidata[1], 1);
        spi_write_blocking(spi0, &spidata[0], 1);
        
        

        trigger_74hc595_stcp();

        if (mma8451q_read_data(&raw_x, &raw_y, &raw_z)) {
            // Convert to g-force
            g_x = convert_to_g(raw_x);
            g_y = convert_to_g(raw_y);
            g_z = convert_to_g(raw_z);

            printf("X: %.3fg, Y: %.3fg, Z: %.3fg\n", g_x, g_y, g_z);
        } else {
            printf("Failed to read accelerometer data.\n");
        }

        // Adjust sleep based ODR to avoid reading too fast or too slow
        // For 100Hz ODR, data is updated every 10ms.
        // Sleep for a bit less than the ODR period to ensure new data is available.
        sleep_ms(100); // Example: sleep for 10ms for 100Hz ODR
        //ht16k33_scroll_string("0   1   2   3   4   5   6   7   8   9   ", 300);
        
        //display_snake(100);
    }
    
    //display_snake(100);

    //ht16k33_display_set(0, 0);
    //ht16k33_display_set(1, 0);
    //ht16k33_display_set(2, 0);
    //ht16k33_display_set(3, 0);

    //ht16k33_scroll_string("0   1   2   3   4   5   6   7   8   9   ", 300);
    /*

    // Do a speeding up propeller effort using the inner segments
    int bits[] = {0x40, 0x0100, 0x0200, 0x0400, 0x80, 0x2000, 0x1000, 0x0800};
    for (int j = 0;j < 10;j++) {
        for (uint i = 0;i< count_of(bits); i++) {
            for (int digit = 0;digit <= NUM_DIGITS; digit++) {
                ht16k33_display_set(digit, bits[i]);
            }
            sleep_ms(155 - j*15);
        }
    }  

    */
    
    /*

    char *strs[] = {
        "Help", "I am", "in a", "Pico", "and ", "Cant", "get ", "out "
    };

    for (uint i = 0; i < count_of(strs); i++) {
        ht16k33_display_string(strs[i]);
        sleep_ms(500);
    }

    */
    

    // Test brightness and blinking

    // Set all segments on all digits on
    //ht16k33_display_set(0, 0xff);
    //ht16k33_display_set(1, 0xff);
    //ht16k33_display_set(2, 0xff);
    //ht16k33_display_set(3, 0xff);

    /*
    // Fade up and down
    for (int j=0;j<5;j++) {
        for (int i = 0; i < 15; i++) {
            ht16k33_set_brightness(i);
            sleep_ms(30);
        }

        for (int i = 14; i >=0; i--) {
            ht16k33_set_brightness(i);
            sleep_ms(30);
        }
    }
    */

    //ht16k33_set_brightness(15);

    //ht16k33_set_blink(1); // 0 for no blink, 1 for 2Hz, 2 for 1Hz, 3 for 0.5Hz
    //sleep_ms(5000);
    //ht16k33_set_blink(0);
}
