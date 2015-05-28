/*
 Contributors:
    Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
*/
/* **************************************************************************
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

// **************************************************************************
//
// Description:          *** HAL for Arduino Due ***
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef _HAL_H
#define _HAL_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

#include "Arduino.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define analogInputToDigitalPin(IO) IO

#define     CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
#define     CRITICAL_SECTION_END    if (primask==0) __enable_irq();

// On AVR this is in math.h?
#define square(x) ((x)*(x))

#define strncpy_P(dest, src, num) strncpy((dest), (src), (num))

#define HIGH 1
#define LOW 0

// --------------------------------------------------------------------------
// magic I/O routines
// now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
// --------------------------------------------------------------------------

#ifndef MASK
  #define MASK(PIN)  (1 << PIN)
#endif

static inline void digitalFastWrite(int pin, bool v) {
  if (v) g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;
  else g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;
}

#define _FASTREAD(IO) ((bool)(DIO ## IO ## _WPORT -> PIO_PDSR & (MASK(DIO ## IO ## _PIN))))

#define _FASTWRITE(IO, v) do {  if (v) {DIO ## IO ## _WPORT -> PIO_SODR |= MASK(DIO ## IO ##_PIN); } \
                                else {DIO ##  IO ## _WPORT -> PIO_CODR = MASK(DIO ## IO ## _PIN); }; \
                             } while (0)

                            
/// Read a pin
//#define   READ(pin)       PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)
#define READ(pin)           _FASTREAD(pin)

/// write to a pin
//#define	  WRITE(pin, v)   do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} \
                          else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define WRITE_VAR(pin, v)   digitalWrite(pin, v)
#define WRITE(pin, v)       _FASTWRITE(pin, v)

/// set pin as input
#define	  SET_INPUT(pin)  pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
                          PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
/// set pin as output
#define	  SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
                          g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
/// toggle a pin	
#define   TOGGLE(pin)     WRITE(pin, !READ(pin))

// Write doesn't work for pullups
#define   PULLUP(pin, v)  {pinMode(pin, (v!=LOW ? INPUT_PULLUP : INPUT));}

/// check if pin is an input
#define   GET_INPUT(pin)

/// check if pin is an output
#define   GET_OUTPUT(pin)

/// check if pin is an timer
#define   GET_TIMER(pin)

// Shorthand
#define   OUT_WRITE(pin, v) {SET_OUTPUT(pin); WRITE_VAR(pin, v);}

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// reset reason set by bootloader
extern uint8_t MCUSR;
static uint32_t tone_pin;
volatile static uint32_t debug_counter;

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------


// Hardware setup
void spiBegin();
void spiInit(uint8_t spiClock);
void spiSendByte(uint32_t chan, byte b);
void spiSend(uint32_t chan ,const uint8_t* buf , size_t n);

// Disable interrupts
void cli(void);

// Enable interrupts
void sei(void);

static inline void _delay_ms(uint32_t msec) {
	delay(msec);
}

static inline void _delay_us(uint32_t usec) {
  uint32_t n = usec * (F_CPU / 3000000);
  asm volatile(
      "L2_%=_delayMicroseconds:"       "\n\t"
      "subs   %0, #1"                 "\n\t"
      "bge    L2_%=_delayMicroseconds" "\n"
      : "+r" (n) :  
  );
}

int freeMemory(void);
void eeprom_write_byte(unsigned char *pos, unsigned char value);
unsigned char eeprom_read_byte(unsigned char *pos);


// timers
#define STEP_TIMER_NUM 2
#define STEP_TIMER_COUNTER TC0
#define STEP_TIMER_CHANNEL 2
#define STEP_TIMER_IRQN TC2_IRQn
#define HAL_STEP_TIMER_ISR 	void TC2_Handler()

#define TEMP_TIMER_NUM 3
#define TEMP_TIMER_COUNTER TC1
#define TEMP_TIMER_CHANNEL 0
#define TEMP_FREQUENCY 2000

#define TEMP_TIMER_IRQN TC3_IRQn
#define HAL_TEMP_TIMER_ISR 	void TC3_Handler()

#define BEEPER_TIMER_NUM 4
#define BEEPER_TIMER_COUNTER TC1
#define BEEPER_TIMER_CHANNEL 1
#define BEEPER_TIMER_IRQN TC4_IRQn
#define HAL_BEEPER_TIMER_ISR  void TC4_Handler()

#define HAL_TIMER_RATE 		     (F_CPU/2)
#define TICKS_PER_NANOSECOND   (HAL_TIMER_RATE)/1000

#define ENABLE_STEPPER_DRIVER_INTERRUPT()	HAL_timer_enable_interrupt (STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()	HAL_timer_disable_interrupt (STEP_TIMER_NUM)

//

void HAL_step_timer_start(void);
void HAL_temp_timer_start (uint8_t timer_num);
void HAL_timer_set_count (Tc *tc, uint32_t channel, uint32_t count);

void HAL_timer_enable_interrupt (uint8_t timer_num);
void HAL_timer_disable_interrupt (uint8_t timer_num);

void HAL_timer_isr_status (Tc *tc, uint32_t channel);
int HAL_timer_get_count (uint8_t timer_num);
uint32_t HAL_timer_get_count_value ();
void HAL_timer_clear (Tc* tc, uint32_t channel);
//

void tone(uint8_t pin, int frequency);
void noTone(uint8_t pin);
//void tone(uint8_t pin, int frequency, long duration);

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

/*
** direct pins
*/

#define DIO0_PIN 8
#define DIO0_WPORT PIOA

#define DIO1_PIN 9
#define DIO1_WPORT PIOA

#define DIO2_PIN 25
#define DIO2_WPORT PIOB

#define DIO3_PIN 28
#define DIO3_WPORT PIOC

#define DIO4_PIN 26
#define DIO4_WPORT PIOC

#define DIO5_PIN 25
#define DIO5_WPORT PIOC

#define DIO6_PIN 24
#define DIO6_WPORT PIOC

#define DIO7_PIN 23
#define DIO7_WPORT PIOC

#define DIO8_PIN 22
#define DIO8_WPORT PIOC

#define DIO9_PIN 21
#define DIO9_WPORT PIOC

#define DIO10_PIN 29
#define DIO10_WPORT PIOC

#define DIO11_PIN 7
#define DIO11_WPORT PIOD

#define DIO12_PIN 8
#define DIO12_WPORT PIOD

#define DIO13_PIN 27
#define DIO13_WPORT PIOB

#define DIO14_PIN 4
#define DIO14_WPORT PIOD

#define DIO15_PIN 5
#define DIO15_WPORT PIOD

#define DIO16_PIN 13
#define DIO16_WPORT PIOA

#define DIO17_PIN 12
#define DIO17_WPORT PIOA

#define DIO18_PIN 11
#define DIO18_WPORT PIOA

#define DIO19_PIN 10
#define DIO19_WPORT PIOA

#define DIO20_PIN 12
#define DIO20_WPORT PIOB

#define DIO21_PIN 13
#define DIO21_WPORT PIOB

#define DIO22_PIN 26
#define DIO22_WPORT PIOB

#define DIO23_PIN 14
#define DIO23_WPORT PIOA

#define DIO24_PIN 15
#define DIO24_WPORT PIOA

#define DIO25_PIN 0
#define DIO25_WPORT PIOD

#define DIO26_PIN 1
#define DIO26_WPORT PIOD

#define DIO27_PIN 2
#define DIO27_WPORT PIOD

#define DIO28_PIN 3
#define DIO28_WPORT PIOD

#define DIO29_PIN 6
#define DIO29_WPORT PIOD

#define DIO30_PIN 9
#define DIO30_WPORT PIOD

#define DIO31_PIN 7
#define DIO31_WPORT PIOA

#define DIO32_PIN 10
#define DIO32_WPORT PIOD

#define DIO33_PIN 1
#define DIO33_WPORT PIOC

#define DIO34_PIN 2
#define DIO34_WPORT PIOC

#define DIO35_PIN 3
#define DIO35_WPORT PIOC

#define DIO36_PIN 4
#define DIO36_WPORT PIOC

#define DIO37_PIN 5
#define DIO37_WPORT PIOC

#define DIO38_PIN 6
#define DIO38_WPORT PIOC

#define DIO39_PIN 7
#define DIO39_WPORT PIOC

#define DIO40_PIN 8
#define DIO40_WPORT PIOC

#define DIO41_PIN 9
#define DIO41_WPORT PIOC

#define DIO42_PIN 19
#define DIO42_WPORT PIOA

#define DIO43_PIN 20
#define DIO43_WPORT PIOA

#define DIO44_PIN 19
#define DIO44_WPORT PIOC

#define DIO45_PIN 18
#define DIO45_WPORT PIOC

#define DIO46_PIN 17
#define DIO46_WPORT PIOC

#define DIO47_PIN 16
#define DIO47_WPORT PIOC

#define DIO48_PIN 15
#define DIO48_WPORT PIOC

#define DIO49_PIN 14
#define DIO49_WPORT PIOC

#define DIO50_PIN 13
#define DIO50_WPORT PIOC

#define DIO51_PIN 12
#define DIO51_WPORT PIOC

#define DIO52_PIN 21
#define DIO52_WPORT PIOB

#define DIO53_PIN 14
#define DIO53_WPORT PIOB

#define DIO54_PIN 16
#define DIO54_WPORT PIOA

#define DIO55_PIN 24
#define DIO55_WPORT PIOA

#define DIO56_PIN 23
#define DIO56_WPORT PIOA

#define DIO57_PIN 22
#define DIO57_WPORT PIOA

#define DIO58_PIN 6
#define DIO58_WPORT PIOA

#define DIO59_PIN 4
#define DIO59_WPORT PIOA

#define DIO60_PIN 3
#define DIO60_WPORT PIOA

#define DIO61_PIN 2
#define DIO61_WPORT PIOA

#define DIO62_PIN 17
#define DIO62_WPORT PIOB

#define DIO63_PIN 18
#define DIO63_WPORT PIOB

#define DIO64_PIN 19
#define DIO64_WPORT PIOB

#define DIO65_PIN 20
#define DIO65_WPORT PIOB

#define DIO66_PIN 15
#define DIO66_WPORT PIOB

#define DIO67_PIN 16
#define DIO67_WPORT PIOB

#define DIO68_PIN 1
#define DIO68_WPORT PIOA

#define DIO69_PIN 0
#define DIO69_WPORT PIOA

#define DIO70_PIN 17
#define DIO70_WPORT PIOA

#define DIO71_PIN 18
#define DIO71_WPORT PIOA

#define DIO72_PIN 30
#define DIO72_WPORT PIOC

#define DIO73_PIN 21
#define DIO73_WPORT PIOA

#define DIO74_PIN 25
#define DIO74_WPORT PIOA

#define DIO75_PIN 26
#define DIO75_WPORT PIOA

#define DIO76_PIN 27
#define DIO76_WPORT PIOA

#define DIO77_PIN 28
#define DIO77_WPORT PIOA

#define DIO78_PIN 23
#define DIO78_WPORT PIOB

#define DIO79_PIN 17
#define DIO79_WPORT PIOA

#define DIO80_PIN 12
#define DIO80_WPORT PIOB

#define DIO81_PIN 8
#define DIO81_WPORT PIOA

#define DIO82_PIN 11
#define DIO82_WPORT PIOA

#define DIO83_PIN 13
#define DIO83_WPORT PIOA

#define DIO84_PIN 4
#define DIO84_WPORT PIOD

#define DIO85_PIN 11
#define DIO85_WPORT PIOB

#define DIO86_PIN 21
#define DIO86_WPORT PIOB

#define DIO87_PIN 29
#define DIO87_WPORT PIOA

#define DIO88_PIN 15
#define DIO88_WPORT PIOB

#define DIO89_PIN 14
#define DIO89_WPORT PIOB

#define DIO90_PIN 1
#define DIO90_WPORT PIOA

#define DIO91_PIN 15
#define DIO91_WPORT PIOB

#define DIO92_PIN 5
#define DIO92_WPORT PIOA

#define DIO93_PIN 12
#define DIO93_WPORT PIOB

#define DIO94_PIN 22
#define DIO94_WPORT PIOB

#define DIO95_PIN 23
#define DIO95_WPORT PIOB

#define DIO96_PIN 24
#define DIO96_WPORT PIOB

#define DIO97_PIN 20
#define DIO97_WPORT PIOC

#define DIO98_PIN 27
#define DIO98_WPORT PIOC

#define DIO99_PIN 10
#define DIO99_WPORT PIOC

#define DIO100_PIN 11
#define DIO100_WPORT PIOC

#endif // _HAL_H
