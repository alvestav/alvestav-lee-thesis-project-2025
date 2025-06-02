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

// spi defines
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SHCP  34
#define PIN_DS 35
#define PIN_STCP 36
#define PIN_OE 37
#define PIN_MR 38

// Accelerometer commands
#define MMA8451Q_addr 1
#define MMA8451Q_OUT_X_MSB 

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

bool mma8451q_init() {
    // return 1 if init passed
    return true;
    // return 0 if init failed

}

void mma8451q_read_register(uint8_t) {

}

void mma8451q_read_data(uint16_t* x, uint16_t* y, uint16_t* z) {
    
}

void trigger_74hc595_stcp() {
    gpio_put(PIN_STCP, true);
    gpio_put(PIN_STCP, false);
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


    //uint16_t raw_x, raw_y, raw_z;
    //float g_x, g_y, g_z;

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

    volatile int btn1, btn2, btn3, btn4;
    volatile int sw1, sw2, sw3, sw4, sw5, sw6, sw7, sw8;

    // start program loop
    // pull-up inverts gpio read, so 'off' switch is read as 1
    while (true)
    {
        btn1 = !gpio_get(BTN1);
        btn2 = !gpio_get(BTN2);
        btn3 = !gpio_get(BTN3);
        btn4 = !gpio_get(BTN4);
        
        

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
            gpio_put(LED_GREEN, 1);
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
