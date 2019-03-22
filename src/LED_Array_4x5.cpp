/**
 * @file    LED_Array_4x5.cpp
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
 
#include <Arduino.h>

#include "LED_Array_4x5.h"

// We need to create a global instance so that the ISR knows what to talk to
LED_Array_4x5 Plex;

/**
 * @brief Define static member for the location of the LEDs
 */
const charlieLed LED_Array_4x5::charlie_leds_[] = { 
											{0,1}, {0,2}, {0,3}, {0,4},
											{1,0}, {1,2}, {1,3}, {1,4},
											{2,0}, {2,1}, {2,3}, {2,4},
											{3,0}, {3,1}, {3,2}, {3,4},
											{4,0}, {4,1}, {4,2}, {4,3}
											};
							
/**
 * @brief Constructor - Instantiates LED array object
 */
LED_Array_4x5::LED_Array_4x5()
{
    // Initialize members
    scrolling_ = 0;
    shift_count_ = 0;
    shift_delay_ = 200; // Arbitrary long wait before scrolling
    scroll_index_ = 0;
    scroll_len_ = 0;

}

/**
 * @brief Destructor
 */
LED_Array_4x5::~LED_Array_4x5()
{
	
}

/**
 * @brief Configures the pins on the Charlieplex array.
 *
 * You must call this function before performing any other actions on the
 * LED array.
 *
 * @param[in] pins Array of pin numbers. Must be 8 bytes long.
 * @return True if array configured. False on error.
 */
bool LED_Array_4x5::init(byte pins[NUM_CHAPLEX_PINS])
{
	
	// Right now, we only compile for the ATtiny85-based Arduinos
#if defined __AVR_ATtiny85__

	// If we are scrolling, stop and delete our string buffer
	if ( scrolling_ )
	{
		stopScrolling();
	}
	
	// Initialize members (again)
	scrolling_ = 0;
    shift_count_ = 0;
    shift_delay_ = DEFAULT_SHIFT_DELAY;
    scroll_index_ = 0;
    scroll_len_ = 0;
	
	// If we already have a Chaplex object, delete it
	if ( chaplex_ != NULL ) 
    {
        delete chaplex_;
    }
	
	// Create a new Chaplex object so we can write stuff to the LEDs
	chaplex_ = new Chaplex(pins, NUM_CHAPLEX_PINS);
	
	// Calculate the Timer 2 reset number. Aim for 2.048 ms refresh.
    // count = 256 - (2.048 ms * F_CPU) / 1024 */
    timer1_count_ = 256 - (2 * (F_CPU / 1000000));  // Aim for 2.048ms refresh
	
	// Initialize Timer 1
    noInterrupts();             			// Disable interrupts
    TCCR1 = 0; 								// Normal operation, clear prescaler
    TCCR1 |= (1 << CS13) | (1 << CS11) | (1 << CS10);	// Prescaler = 1024
	TCNT1 = timer1_count_;      			// Load counter
    TIMSK |= (1 << TOIE1);      			// Enable timer overflow
    interrupts();               			// Enable all interrupts

    // Clear and load frame buffer
    clear();
    display();
    
    return true;
	
#else	// if defined __AVR_ATtiny85__

    return false;
    
#endif	// if defined __AVR_ATtiny85__
}

/**
 * @brief Writes the frame buffer to the LED buffer.
 */
void LED_Array_4x5::display()
{
    for ( byte i = 0; i < NUM_LEDS; i++ ) 
    {
        chaplex_->ledWrite(charlie_leds_[i], frame_buffer_[i]);
    }
}

/**
 * @brief Clears the Charlieplex array.
 */
void LED_Array_4x5::clear()
{
    memset(frame_buffer_, 0, NUM_LEDS);
}

/**
 * @brief Turns a pixel at a given (x, y) on or off
 *
 * Coordinates start (0, 0) from the top-left of the display.
 *
 * @param[in] x X coordinate for the pixel
 * @param[in] y Y coordinate for the pixel
 * @param[in] on 1 for on, 0 for off.
 */
void LED_Array_4x5::pixel(uint8_t x, uint8_t y, uint8_t on /* = 1 */)
{
    // Check to make sure that we are not accessing outside the array
    if ( x >= ROW_SIZE || y >= COL_SIZE ) 
    {
        return;
    }
    
    // Turn the specified LED on or off. Note that we need to switch our X and Y
    // for the user, as X goes down and Y goes across on the actual LED display.
    if ( on ) 
    {
		frame_buffer_[(y * ROW_SIZE) + x] = 1;
    }
    else 
    {
        frame_buffer_[(y * ROW_SIZE) + x] = 0;
    }
}

// ***TODO: rest of drawing and scrolling text

/**
 * @brief Stops scrolling text and deletes scroll buffer
 */
void LED_Array_4x5::stopScrolling()
{
    //***TODO: Implement this
}

/**
 * @brief Returns the width of the LED array
 *
 * @return width of the array (number of LEDs)
 */
uint8_t LED_Array_4x5::getArrayWidth()
{
    return ROW_SIZE;
}

/**
 * @brief Returns the width of the LED array
 *
 * @return width of the array (number of LEDs)
 */
uint8_t LED_Array_4x5::getArrayHeight()
{
    return COL_SIZE;
}

/**
 * @brief Returns the byte at the specified location from the font PROGMEM
 *
 * @param[in] idx the index for the character
 * @param[in] offset number of bytes off from the beginning of the character
 * @return byte at the specified location
 */
unsigned char LED_Array_4x5::getPGMFontByte(int idx, int offset /* = 0 */)
{
    //***TODO: return pgm_read_byte(pgm_read_word(&char_table[idx]) + offset);
	return false;
}

/**
 * @brief Swaps the given bytes
 *
 * @param[in, out] a first byte (becomes b)
 * @param[in, out] b second byte (becomes a)
 */
void LED_Array_4x5::swap(uint8_t &a, uint8_t &b) 
{
    uint8_t t = a;
    a = b;
    b = t;
}

#if defined __AVR_ATtiny85__

void LED_Array_4x5::isr()
{

    // Disable Timer1 interrupts
    TIMSK &= ~(1 << TOIE1);

    // Shift one column
    if ( scrolling_ ) 
    {
        shift_count_++;
        if ( shift_count_ >= shift_delay_ ) 
        {
            shift_count_ = 0;
            byte i;
            byte bit_to_shift;
            
            // Shift all but last column
            for ( i = 0; i < ALL_BUT_LAST_COL; i++ ) 
            {
                frame_buffer_[i] = frame_buffer_[i + COL_SIZE];
            }
            
            // Shift in new column at the end
            for ( i = 0; i < COL_SIZE; i++ ) 
            {
                bit_to_shift = (scroll_buf_[scroll_index_] >> i) & 0x01;
                frame_buffer_[ALL_BUT_LAST_COL + i] = bit_to_shift;
            }
            
            // Send everything in our new buffer to the LED matrix
            display();
            
            // Increment buffer index and reset if it reaches the end
            scroll_index_++;
            if ( scroll_index_ >= scroll_len_ ) 
            {
                scroll_index_ = 0;
                if ( scroll_times_ > 0 ) 
                {
                    scroll_count_++;
                    if ( scroll_count_ >= scroll_times_ ) 
                    {
                        stopScrolling();
                    }
                }
            }
        }
    }

    // Display a row and reset counter
    chaplex_->outRow();            // Output for 1 LED row
    TCNT1 = timer1_count_;         // Load counter for next interrupt
    TIMSK |= (1 << TOIE1);         // Enable timer overflow interrupt
}

/**
 * @brief Global interrupt service routine for Timer 1
 *
 * We define Timer 1 ISR here to allow us to make calls to functions in the
 * LED_Array_4x5 class. To do this, we instantiate an LED_Array_4x5 object
 * (globally) in the .cpp file.
 **/
ISR(TIMER1_OVF_vect) 
{
    Plex.isr();
}

#else	// if defined __AVR_ATtiny85__

void LED_Array_4x5::isr()
{

}

#endif	// if defined __AVR_ATtiny85__