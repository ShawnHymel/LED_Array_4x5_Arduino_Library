/**
 * @file    LED_Array_4x5.h
 * @brief   Library for the Plex85 board
 * @author  Shawn Hymel
 * @date    March 3, 2019
 *
 * This library controls the 4x5 Charlieplex LED array. Note that Timer1 is used
 * in this library to control the LED refresh. You will not be able to use it
 * for other uses.
 *
 * This library is based entirely on the SparkFun LED Array 8x7 Arduino library:
 * https://github.com/sparkfun/SparkFun_LED_Array_8x7_Arduino_Library
 *
 * The graphics algorithms are based on Jim Lindblom's Micro OLED library:
 * https://github.com/sparkfun/Micro_OLED_Breakout
 *
 * Relies on the Chaplex library written by Stefan Götze. Download and install
 * Chaplex.zip from https://code.google.com/archive/p/yacll/downloads
 *
 * @copyright The MIT License (MIT)
 * 
 * Copyright (c) 2019 Shawn Hymel
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#ifndef LED_Array_4x5_H
#define LED_Array_4x5_H

#include <Arduino.h>
#include <Chaplex.h>

//***TODO #include "LED_Font_1.h"

// Debug
#define LED_Array_4x5_DEBUG   0

// Constants
static const unsigned int NUM_CHAPLEX_PINS = 5; // Number of pins  
static const unsigned int DEFAULT_SHIFT_DELAY = 50; // Ticks before scrolling
static const unsigned int MAX_CHARS = 100;      // Maximum characters to scroll
static const unsigned int CHAR_OFFSET = 0x20;   // Starting place for ASCII
static const unsigned int CHAR_SPACE = 1;       // Number columns between chars
static const unsigned int END_SPACE = 8;        // Number columns after text
static const unsigned int COL_SIZE = 5;         // Number LEDs in a column
static const unsigned int ROW_SIZE = 4;         // Number LEDs in a row

// Derived constants
static const unsigned int NUM_LEDS = COL_SIZE * ROW_SIZE;
static const unsigned int ALL_BUT_LAST_COL = NUM_LEDS - COL_SIZE;

// Global variables 
ISR(TIMER1_OVF_vect);

// LED Array class
class LED_Array_4x5 {
	
	// The ISR is our friend! It can call our functions
	friend void TIMER1_OVF_vect();
	
public:

	// Initialization
	LED_Array_4x5();
	~LED_Array_4x5();
	bool init(byte pins[NUM_CHAPLEX_PINS]);
	
	// LED drawing methods
	void display();
	void clear();
	void pixel(uint8_t x, uint8_t y, uint8_t on = 1);
	
	// ***TODO: rest of drawing and scrolling text
	
	// Support methods
	uint8_t getArrayWidth();
	uint8_t getArrayHeight();
	
private:

	// Helper functions
	unsigned char getPGMFontByte(int idx, int offset = 0);
	void swap(uint8_t &a, uint8_t &b);
	
	// Interrupt service routine that is called by system's ISR
	inline void isr();
	
	// Members
    Chaplex *chaplex_;            /// Chaplex object for controlling the LEDs
    byte frame_buffer_[NUM_LEDS]; /// Storing the state of each LED
    uint8_t timer1_count_;        /// Stores the next start point for Timer1
    byte *scroll_buf_;            /// Buffer of text graphics to scroll
    volatile byte scrolling_;     /// Indicates if we are scrolling text
    unsigned int shift_count_;    /// Count number of ticks before shifting text
    unsigned int shift_delay_;    /// Number ticks to wait before shifting text
    unsigned int scroll_index_;   /// Index of where to scroll text
    unsigned int scroll_len_;     /// Number of bytes in scroll_buf_
    unsigned int scroll_times_;   /// Number of times to scroll text
    unsigned int scroll_count_;   /// Counter for times text has scrolled
    static const charlieLed charlie_leds_[]; /// Relative location of the LEDs
};

// Declare a global instance of our object
extern LED_Array_4x5 Plex;

#endif // LED_Array_4x5_H