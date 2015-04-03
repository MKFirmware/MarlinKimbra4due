/****************************************************************************************
* Arduino Due supports the following shields:
*
*401 BOARD_RADDS - Radds Arduino DUE
*403 BOARD_RAMPS_FD_V1 - Ramps FD version 1 Arduino DUE
*404 BOARD_RAMPS_FD_V2 - Ramps FD version 2 Arduino DUE
*
*501 BOARD_ALLIGATOR - Alligator R2 ARM 32
*
*99 BOARD_99 - Custom motherboard
****************************************************************************************/

#ifndef PINS_H
#define PINS_H

#include "boards.h"

// Preset optional pins
#define X_MS1_PIN     -1
#define X_MS2_PIN     -1
#define Y_MS1_PIN     -1
#define Y_MS2_PIN     -1
#define Z_MS1_PIN     -1
#define Z_MS2_PIN     -1
#define E0_MS1_PIN    -1
#define E0_MS2_PIN    -1
#define E1_MS1_PIN    -1
#define E1_MS2_PIN    -1
#define DIGIPOTSS_PIN -1
#define LCD_CONTRAST  -1



/****************************************************************************************
* Arduino Due pin assignment
*
* for RADDS
****************************************************************************************/
#if MB(RADDS)
#define KNOWN_BOARD 1
//
#ifndef __SAM3X8E__
 #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif
//
#define RADDS

#define X_STEP_PIN         24
#define X_DIR_PIN          23
#define X_ENABLE_PIN       26


#define Y_STEP_PIN         17
#define Y_DIR_PIN          16
#define Y_ENABLE_PIN       22

#define Z_STEP_PIN         2
#define Z_DIR_PIN          3
#define Z_ENABLE_PIN       15

#define X_MIN_PIN          28
#define X_MAX_PIN          -1  // 34   //Max endstops default to disabled "-1", set to commented value to enable.
#define Y_MIN_PIN          30
#define Y_MAX_PIN          -1  // 36
#define Z_MIN_PIN          32
#define Z_MAX_PIN          -1  // 38

#define E0_STEP_PIN        61
#define E0_DIR_PIN         60
#define E0_ENABLE_PIN      62

#define E1_STEP_PIN        64
#define E1_DIR_PIN         63
#define E1_ENABLE_PIN      65

#define E2_STEP_PIN        51
#define E2_DIR_PIN         53
#define E2_ENABLE_PIN      49

#define SDPOWER            -1
#define SDSS               4 //10 Display
#define LED_PIN            -1

#define BEEPER             41

#define FAN_PIN            -1

//#define CONTROLLERFAN_PIN  8 //Pin used for the fan to cool controller

#define PS_ON_PIN          40

#define KILL_PIN           -1

#define HEATER_BED_PIN     7    // BED
#define HEATER_0_PIN       13
#define HEATER_1_PIN       12
#define HEATER_2_PIN       11

#define TEMP_BED_PIN       4   // ANALOG NUMBERING
#define TEMP_0_PIN         0   // ANALOG NUMBERING
#define TEMP_1_PIN         -1  // 1   // ANALOG NUMBERING
#define TEMP_2_PIN         -1  // 2   // ANALOG NUMBERING
#define TEMP_3_PIN         -1  // 3   // ANALOG NUMBERING



  #ifdef NUM_SERVOS
    #define SERVO0_PIN         5

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         39
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         40
    #endif
  #endif


#ifdef ULTRA_LCD

	// RADDS LCD panel
	#ifdef NEWPANEL
	  #define LCD_PINS_RS 		42
	  #define LCD_PINS_ENABLE 	43
	  #define LCD_PINS_D4 		44
	  #define LCD_PINS_D5 		45
	  #define LCD_PINS_D6 		46
	  #define LCD_PINS_D7 		47

	  #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
			#define BEEPER 41

			#define BTN_EN1 52
			#define BTN_EN2 50
			#define BTN_ENC 48

			#define SDCARDDETECT 14
	  #endif
	#endif

#endif //ULTRA_LCD


// SPI for Max6675 Thermocouple

//works with radds??? #ifndef SDSUPPORT
//// these pins are defined in the SD library if building with SD support
//  #define MAX_SCK_PIN          52
//  #define MAX_MISO_PIN         50
//  #define MAX_MOSI_PIN         51
//  #define MAX6675_SS       53
//#else
//  #define MAX6675_SS       49
//#endif

#endif //RADDS
/****************************************************************************************/



/****************************************************************************************
* 403 - 404
* Arduino Due pin assignment
* Ramps - FD v1 & v2
****************************************************************************************/
#if MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2)
#define KNOWN_BOARD 1

#ifndef __SAM3X8E__
 #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif


//
#if MB(RAMPS_FD_V1)
  #define INVERTED_HEATER_PINS
  #define RAMPS_FD_V1
  #define INVERTED_HEATER_PINS
  // No EEPROM
  // Use 4k7 thermistor tables
#else
  #define RAMPS_FD_V2
  // EEPROM supported
  // Use 1k thermistor tables
#endif

#define X_STEP_PIN         63
#define X_DIR_PIN          62
#define X_ENABLE_PIN       48
#define X_MIN_PIN          22
#define X_MAX_PIN           2

#define Y_STEP_PIN         65
#define Y_DIR_PIN          64
#define Y_ENABLE_PIN       46
#define Y_MIN_PIN          24
#define Y_MAX_PIN          15

#define Z_STEP_PIN         67
#define Z_DIR_PIN          66
#define Z_ENABLE_PIN       44
#define Z_MIN_PIN          26
#define Z_MAX_PIN          -1

#define E0_STEP_PIN        36
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      42

#define E1_STEP_PIN        43
#define E1_DIR_PIN         41
#define E1_ENABLE_PIN      39

#define E2_STEP_PIN        32
#define E2_DIR_PIN         47
#define E2_ENABLE_PIN      45

#define SDPOWER            -1
#define SDSS               4
#define LED_PIN            13

#define BEEPER             -1

#define FAN_PIN            -1

#define CONTROLLERFAN_PIN  -1 //Pin used for the fan to cool controller

#define PS_ON_PIN          -1

#define KILL_PIN           -1


#define HEATER_BED_PIN     8    // BED

#define HEATER_0_PIN       9
#define HEATER_1_PIN       10
#define HEATER_2_PIN       11

#define TEMP_BED_PIN       0   // ANALOG NUMBERING

#define TEMP_0_PIN         1   // ANALOG NUMBERING
#define TEMP_1_PIN         -1  // 2    // ANALOG NUMBERING
#define TEMP_2_PIN         -1  // 3     // ANALOG NUMBERING

#define TEMP_3_PIN         -1   // ANALOG NUMBERING
#define TEMP_4_PIN         -1   // ANALOG NUMBERING



  #ifdef NUM_SERVOS
    #define SERVO0_PIN         11

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         4
    #endif
  #endif

  #ifdef ULTRA_LCD

    #ifdef NEWPANEL
      // ramps-fd lcd adaptor
      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29

      #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
        #define BEEPER 37

        #define BTN_EN1 33
        #define BTN_EN2 31
        #define BTN_ENC 35

        #define SDCARDDETECT 49
        #endif

      #endif

  #endif //ULTRA_LCD


// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define MAX_SCK_PIN          52
  #define MAX_MISO_PIN         50
  #define MAX_MOSI_PIN         51
  #define MAX6675_SS       53
#else
  #define MAX6675_SS       49
#endif

#endif //RAMPS-FD
/****************************************************************************************/




/****************************************************************************************
* 501
* Arduino Due pin assignment
* Alligator R2
****************************************************************************************/
#if MB(ALLIGATOR)
#define KNOWN_BOARD 1

#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif

#define ALLIGATOR
#define SPI_CHAN_DAC 1

// X AXIS
#define X_STEP_PIN      96  // PB24
#define X_DIR_PIN        2  // PB25
#define X_ENABLE_PIN    24  // PA15, motor RESET pin
#define X_MIN_PIN       33  // PC1
#define X_MAX_PIN       34  // PC2
#define X_MS1_PIN       99  // PC10

// Y AXIS
#define Y_STEP_PIN      94  // PB22
#define Y_DIR_PIN       95  // PB23
#define Y_ENABLE_PIN    24  // PA15, motor RESET pin
#define Y_MIN_PIN       35  // PC3
#define Y_MAX_PIN       37  // PC5
#define Y_MS1_PIN       10  // PC29

// Z AXIS
#define Z_STEP_PIN      98  // PC27
#define Z_DIR_PIN        3  // PC28
#define Z_ENABLE_PIN    24  // PA15, motor RESET pin
#define Z_MIN_PIN       38  // PC6
#define Z_MAX_PIN       39  // PC7
#define Z_MS1_PIN       44  // PC19

// E AXIS
#define E0_STEP_PIN      5  // PC25
#define E0_DIR_PIN       4  // PC26
#define E0_ENABLE_PIN   24  // PA15, motor RESET pin
#define E0_MS1_PIN      45  // PC18

/*
#define E1_STEP_PIN     28  // PD3 on piggy
#define E1_DIR_PIN      27  // PD2 on piggy
#define E1_ENABLE_PIN   -1
#define E1_MS1_PIN      -1

#define E2_STEP_PIN     11 // PD7 on piggy
#define E2_DIR_PIN      29 // PD6 on piggy
#define E2_ENABLE_PIN   -1
#define E2_MS_PIN       -1

#define E3_STEP_PIN     30 // PD9 on piggy
#define E3_DIR_PIN      12 // PD8 on piggy
#define E3_ENABLE_PIN   -1
#define E3_MS_PIN       -1
*/

#define MOTOR_FAULT_PIN 22 // PB26 , motor X-Y-Z-E0 motor FAULT

#define SDPOWER 	      -1
#define SDSS            77 // PA28
#define SDCARDDETECT    87 // PA29
#define SDCARDDETECTINVERTED false
#define LED_PIN 	      -1

#define FAN_PIN         92 // PA5
#define FAN2_PIN        31 // PA7

#define PS_ON_PIN       -1
#define KILL_PIN        -1
#define SUICIDE_PIN     -1 //PIN that has to be turned on right after start, to keep power flowing.
#define HEAT_OFF_INT_PIN 50 // PC13 on raspberry expansion


// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_BED_PIN  69 // PA0
#define HEATER_0_PIN    68 // PA1
/*
#define HEATER_1_PIN     8  // PC22 on piggy
#define HEATER_2_PIN     9  // PC21 on piggy
#define HEATER_3_PIN    97 // PC20 on piggy
*/

#define TEMP_BED_PIN     0 // PA16
#define TEMP_0_PIN       1  // PA24, analog pin
/*
#define TEMP_1_PIN       5  // PA23 analog pin on piggy
#define TEMP_2_PIN       4  // PA22, analog pin on piggy
#define TEMP_3_PIN       3  // PA6, analog on piggy
*/

#define LED_RED_PIN     40 // PC8
#define LED_GREEN_PIN   41 // PC9
#define CASE_LIGHTS_PIN 36 // PC4

#define EXP_VOLTAGE_LEVEL_PIN 65

#define DAC_SYNC        53 // PB14

//64K SPI EEPROM
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS  25 // PD0

//2K SPI EEPROM
#define SPI_EEPROM2_CS  26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS    23 //PA14


#endif //ALLIGATOR
/****************************************************************************************/


/****************************************************************************************
*********** Available chip select pins for HW SPI are 4 10 52 ***************************
/****************************************************************************************/
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 77)
  #if (SDSS == 4)
    #define SPI_PIN         87
    #define SPI_CHAN        1
  #elif (SDSS == 10)
    #define SPI_PIN         77
    #define SPI_CHAN        0
  #elif (SDSS == 52) 
    #define SPI_PIN         86
    #define SPI_CHAN        2
  #else
    #define SPI_PIN         77
    #define SPI_CHAN        0
  #endif
  #define MOSI_PIN          75
  #define MISO_PIN          74
  #define SCK_PIN           76
//#define DUE_SOFTWARE_SPI
#else
  #define DUE_SOFTWARE_SPI
  #define MOSI_PIN		      51
  #define MISO_PIN		      50
  #define SCK_PIN 		      52
#endif
/****************************************************************************************/


/****************************************************************************************
********************************* END MOTHERBOARD ***************************************
/****************************************************************************************/

#ifndef KNOWN_BOARD
  #error Unknown MOTHERBOARD value in configuration.h
#endif

#ifndef HEATER_1_PIN
  #define HEATER_1_PIN -1
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN -1
#endif
#ifndef HEATER_2_PIN
  #define HEATER_2_PIN -1
#endif
#ifndef TEMP_2_PIN
  #define TEMP_2_PIN -1
#endif
#ifndef HEATER_3_PIN
  #define HEATER_3_PIN -1
#endif
#ifndef TEMP_3_PIN
  #define TEMP_3_PIN -1
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#if X_HOME_DIR > 0        //Home X to MAX
  #define X_MIN_PIN          -1
#elif X_HOME_DIR < 0      //Home X to MIN
  #define X_MAX_PIN          -1
#endif //X_HOME_DIR > 0

#if Y_HOME_DIR > 0        //Home Y to MAX
  #define Y_MIN_PIN          -1
#elif Y_HOME_DIR < 0      //Home Y to MIN
  #define Y_MAX_PIN          -1
#endif //Y_HOME_DIR > 0

#ifndef DELTA
  #if Z_HOME_DIR > 0      //Home Z to MAX
    #define Z_MIN_PIN        -1
  #elif Z_HOME_DIR < 0    //Home Z to MIN
    #define Z_MAX_PIN        -1
  #endif //Z_HOME_DIR > 0
#endif //!DELTA
/****************************************************************************************/



/****************************************************************************************
************************************* FEATURE *******************************************
****************************************************************************************/

#if HOTENDS == 1
  #undef HEATER_1_PIN
  #undef HEATER_2_PIN
  #undef HEATER_3_PIN
  #define HEATER_1_PIN  -1
  #define HEATER_2_PIN  -1
  #define HEATER_3_PIN  -1
  #undef TEMP_1_PIN
  #undef TEMP_2_PIN
  #undef TEMP_3_PIN
  #define TEMP_1_PIN    -1
  #define TEMP_2_PIN    -1
  #define TEMP_3_PIN    -1
#elif HOTENDS == 2
  #undef HEATER_2_PIN
  #undef HEATER_3_PIN
  #define HEATER_2_PIN  -1
  #define HEATER_3_PIN  -1
  #undef TEMP_2_PIN
  #undef TEMP_3_PIN
  #define TEMP_2_PIN    -1
  #define TEMP_3_PIN    -1
#elif HOTENDS == 3
  #undef HEATER_3_PIN
  #define HEATER_3_PIN  -1
  #undef TEMP_3_PIN
  #define TEMP_3_PIN    -1
#endif

#ifdef MKR4
  #if (EXTRUDERS == 2) && (DRIVER_EXTRUDERS==1)     // Use this for one driver and two extruder
    #define E0E1_CHOICE_PIN    40
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS==1)     // Use this for one driver and 3 extruder
    #define E0E1_CHOICE_PIN    40
    #define E0E2_CHOICE_PIN    41
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS==1)     // Use this for one driver and 4 extruder
    #define E0E1_CHOICE_PIN    40
    #define E0E2_CHOICE_PIN    41
    #define E0E3_CHOICE_PIN    42
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS==2)   // Use this for two driver and 3 extruder
    #define E0E2_CHOICE_PIN    5
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS==2)   // Use this for two driver and 4 extruder
    #define E0E2_CHOICE_PIN    5
    #define E1E3_CHOICE_PIN    6
  #endif //EXTRUDERS
#endif //MKR4

#ifdef NPR2
  #define E_MIN_PIN           19
#endif //NPR2

#ifdef LASERBEAM
  #define LASER_PWR_PIN	42
  #define LASER_TTL_PIN	44
#endif

#ifdef FILAMENT_END_SWITCH
  #define PAUSE_PIN           19
#endif

#ifdef FILAMENT_SENSOR
  #define FILWIDTH_PIN          5   // ANALOG NUMBERING
#endif
  
#ifdef POWER_CONSUMPTION
  #define POWER_CONSUMPTION_PIN 4   // ANALOG NUMBERING
#endif
/****************************************************************************************/


#include "pins2tool.h"

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if DRIVER_EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if DRIVER_EXTRUDERS  > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif
#if DRIVER_EXTRUDERS > 3
  #define _E3_PINS E3_STEP_PIN, E3_DIR_PIN, E3_ENABLE_PIN, HEATER_3_PIN,
#else
  #define _E3_PINS
#endif

#define SENSITIVE_PINS { 0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN, \
                        _E0_PINS _E1_PINS _E2_PINS _E3_PINS \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_3_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }

#define HAS_DIGIPOTSS (DIGIPOTSS_PIN >= 0)

#endif //__PINS_H
