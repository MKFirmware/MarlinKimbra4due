/****************************************************************************************
* 701
* MegaTronics v2.0
****************************************************************************************/

  #define KNOWN_BOARD 1

  #ifndef __AVR_ATmega2560__
    #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH        true

  #define ORIG_X_STEP_PIN 26
  #define ORIG_X_DIR_PIN 27
  #define ORIG_X_ENABLE_PIN 25
  #define ORIG_X_MIN_PIN 37
  #define ORIG_X_MAX_PIN 40 //2 //Max endstops default to disabled "-1", set to commented value to enable.

  #define ORIG_Y_STEP_PIN 4 // A6
  #define ORIG_Y_DIR_PIN 54 // A0
  #define ORIG_Y_ENABLE_PIN 5
  #define ORIG_Y_MIN_PIN 41
  #define ORIG_Y_MAX_PIN 38 //15

  #define ORIG_Z_STEP_PIN 56 // A2
  #define ORIG_Z_DIR_PIN 60 // A6
  #define ORIG_Z_ENABLE_PIN 55 // A1
  #define ORIG_Z_MIN_PIN 18
  #define ORIG_Z_MAX_PIN 19

  #define ORIG_E0_STEP_PIN 35
  #define ORIG_E0_DIR_PIN 36
  #define ORIG_E0_ENABLE_PIN 34

  #define ORIG_E1_STEP_PIN 29
  #define ORIG_E1_DIR_PIN 39
  #define ORIG_E1_ENABLE_PIN 28

  #define ORIG_E2_STEP_PIN 23
  #define ORIG_E2_DIR_PIN 24
  #define ORIG_E2_ENABLE_PIN 22

  #define SDPOWER -1
  #define SDSS 53
  #define LED_PIN 13

  #define ORIG_FAN_PIN 7
  #define ORIG_FAN2_PIN 6
  #define ORIG_PS_ON_PIN 12
  #define KILL_PIN -1

  #define ORIG_HEATER_0_PIN 9 // EXTRUDER 1
  #define ORIG_HEATER_1_PIN 8 // EXTRUDER 2
  #define ORIG_HEATER_2_PIN -1

  #define SHIFT_CLK 63
  #define SHIFT_LD 42
  #define SHIFT_OUT 17
  #define SHIFT_EN 17

  #if TEMP_SENSOR_0 == -1
    #define ORIG_TEMP_0_PIN 4 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_0_PIN 13 // ANALOG NUMBERING
  #endif

  #if TEMP_SENSOR_1 == -1
    #define ORIG_TEMP_1_PIN 8 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_1_PIN 15 // ANALOG NUMBERING
  #endif

  #define ORIG_TEMP_2_PIN -1 // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN 10 // BED

  #if TEMP_SENSOR_BED == -1
    #define ORIG_TEMP_BED_PIN 8 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_BED_PIN 14 // ANALOG NUMBERING
  #endif

  #define ORIG_BEEPER_PIN 64

  #define LCD_PINS_RS 14
  #define LCD_PINS_ENABLE 15
  #define LCD_PINS_D4 30
  #define LCD_PINS_D5 31
  #define LCD_PINS_D6 32
  #define LCD_PINS_D7 33

  //buttons are directly attached using keypad
  #define BTN_EN1 61
  #define BTN_EN2 59
  #define BTN_ENC 43 //the click

  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0

  #define SD_DETECT_PIN -1  // Megatronics does not use this port

