/**
 * This is the main Hardware Abstraction Layer (HAL).
 * To make the firmware work with different processors and toolchains,
 * all hardware related code should be packed into the hal files.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Description:          *** HAL for Arduino Due ***
 *
 * ARDUINO_ARCH_SAM
 */

#ifndef HAL_H
  #define HAL_H

  #include <stdint.h>
  #include "Arduino.h"
  #include "fastio.h"

  /**
   * Defines & Macros
   */
  // Compiler warning on unused varable.
  #define UNUSED(x) (void) (x)

  // Macros for bit
  #define BIT(b)                (1<<(b))
  #define TEST(n, b)            (((n)&BIT(b))!=0)
  #define SET_BIT(n, b, value)  (n) ^= ((-value)^(n)) & (BIT(b))

  // Macros for maths shortcuts
  #ifndef M_PI 
    #define M_PI 3.1415926536
  #endif
  #define RADIANS(d) ((d)*M_PI/180.0)
  #define DEGREES(r) ((r)*180.0/M_PI)
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  #define square(x) ((x)*(x))
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))

  // Macros to support option testing
  #define PIN_EXISTS(PN) (defined(PN##_PIN) && PN##_PIN >= 0)
  #define HAS(FE) (HAS_##FE)
  #define HASNT(FE) (!(HAS_##FE))

  // Macros to contrain values
  #define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
  #define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
  #define COUNT(a) (sizeof(a)/sizeof(*a))

  #define analogInputToDigitalPin(IO) IO
  #define FORCE_INLINE __attribute__((always_inline)) inline

  #define CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
  #define CRITICAL_SECTION_END    if (primask==0) __enable_irq();

  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
    #define COMPAT_PRE1
  #endif

  /**
   * Types
   */
  typedef char prog_char;
  typedef uint32_t millis_t;

  /**
   * Public Variables
   */

  #ifndef DUE_SOFTWARE_SPI
    extern int spiDueDividors[];
  #endif

  // reset reason set by bootloader
  extern uint8_t MCUSR;
  volatile static uint32_t debug_counter;

  /**
   * Setting Serial
   */
  #if SERIAL_PORT == -1
    #define MKSERIAL SerialUSB
  #elif SERIAL_PORT == 0
    #define MKSERIAL Serial
  #elif SERIAL_PORT == 1
    #define MKSERIAL Serial1
  #elif SERIAL_PORT == 2
    #define MKSERIAL Serial2
  #elif SERIAL_PORT == 3
    #define MKSERIAL Serial3
  #endif

  #if defined(BLUETOOTH) && BLUETOOTH_PORT > 0
    #undef MKSERIAL
    #if BLUETOOTH_PORT == 1
      #define MKSERIAL Serial1
    #elif BLUETOOTH_PORT == 2
      #define MKSERIAL Serial2
    #elif BLUETOOTH_PORT == 3
      #define MKSERIAL Serial3
    #endif
  #endif

  class HAL {
    public:

      HAL();

      virtual ~HAL();

      static void showStartReason();
      static int getFreeRam();
      static void resetHardware();
    protected:
    private:
  };

  #ifdef DUE_SOFTWARE_SPI
    inline uint8_t spiTransfer(uint8_t b); // using Mode 0
    inline void spiBegin();
    inline void spiInit(uint8_t spiClock);
    inline uint8_t spiReceive();
    inline void spiReadBlock(uint8_t*buf, uint16_t nbyte);
    inline void spiSend(uint8_t b);
    inline void spiSend(const uint8_t* buf , size_t n) ;
    inline void spiSendBlock(uint8_t token, const uint8_t* buf);
  #else
    // Hardware setup
    void spiBegin();
    void spiInit(uint8_t spiClock);
    // Write single byte to SPI
    void spiSend(byte b);
    void spiSend(const uint8_t* buf, size_t n);
    void spiSend(uint32_t chan, byte b);
    void spiSend(uint32_t chan , const uint8_t* buf , size_t n);
    // Read single byte from SPI
    uint8_t spiReceive();
    uint8_t spiReceive(uint32_t chan);
    // Read from SPI into buffer
    void spiReadBlock(uint8_t* buf, uint16_t nbyte);
    // Write from buffer to SPI
    void spiSendBlock(uint8_t token, const uint8_t* buf);
  #endif

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
  void eeprom_write_byte(unsigned char* pos, unsigned char value);
  unsigned char eeprom_read_byte(unsigned char* pos);

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

  void HAL_step_timer_start(void);
  void HAL_temp_timer_start (uint8_t timer_num);

  void HAL_timer_enable_interrupt (uint8_t timer_num);
  void HAL_timer_disable_interrupt (uint8_t timer_num);

  inline
  void HAL_timer_isr_status (Tc* tc, uint32_t channel) {
    tc->TC_CHANNEL[channel].TC_SR; // clear status register
  }

  int HAL_timer_get_count (uint8_t timer_num);
  //

  void tone(uint8_t pin, int frequency);
  void noTone(uint8_t pin);
  //void tone(uint8_t pin, int frequency, long duration);

  uint16_t getAdcReading(adc_channel_num_t chan);
  void startAdcConversion(adc_channel_num_t chan);
  adc_channel_num_t pinToAdcChannel(int pin);

  uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion = false);
  uint16_t getAdcSuperSample(adc_channel_num_t chan);
  void stopAdcFreerun(adc_channel_num_t chan);

#endif // HAL_H
