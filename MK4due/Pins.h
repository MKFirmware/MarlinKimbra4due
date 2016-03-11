/**
 * pins.h
 */

#ifndef PINS_H
#define PINS_H


// #define KNOWN_BOARD

#if MB(GEN7_CUSTOM)
    #include "module/boards/pins_GEN7_CUSTOM.h"
#elif MB(GEN7_12)
    #include "module/boards/pins_GEN7_12.h"
#elif MB(GEN7_13)
    #include "module/boards/pins_GEN7_13.h"
#elif MB(GEN7_14)
    #include "module/boards/pins_GEN7_14.h"
#elif MB(CHEAPTRONIC)
    #include "module/boards/pins_CHEAPTRONIC.h"
#elif MB(SETHI)
    #include "module/boards/pins_SETHI.h"
#elif MB(ELEFU_3)
    #include "module/boards/pins_ELEFU_3.h"
#elif MB(GEN3_MONOLITHIC)
    #include "module/boards/pins_GEN3_MONOLITHIC.h"
#elif MB(RAMPS_OLD)
    #include "module/boards/pins_RAMPS_OLD.h"
#elif MB(RAMPS_13_HFB)
    #include "module/boards/pins_RAMPS_13_HFB.h"
#elif MB(RAMPS_13_HHB)
    #include "module/boards/pins_RAMPS_13_HHB.h"
#elif MB(RAMPS_13_HFF)
    #include "module/boards/pins_RAMPS_13_HFF.h"
#elif MB(RAMPS_13_HHF)
    #include "module/boards/pins_RAMPS_13_HHF.h"
#elif MB(RAMPS_13_HHH)
    #include "module/boards/pins_RAMPS_13_HHH.h"
#elif MB(RAMBO)
    #include "module/boards/pins_RAMBO.h"
#elif MB(PIBOT)
    #include "module/boards/pins_PIBOT.h"
#elif MB(DUEMILANOVE_328P)
    #include "module/boards/pins_DUEMILANOVE_328P.h"
#elif MB(MKS_BASE)
    #include "module/boards/pins_MKS_BASE.h"
#elif MB(MKS_MINI)
    #include "module/boards/pins_MKS_MINI.h"
#elif MB(RADDS)
    #include "module/boards/pins_RADDS.h"
#elif MB(RAMPS_FD_V1)
    #include "module/boards/pins_RAMPS_FD.h"
#elif MB(RAMPS_FD_V2)
    #include "module/boards/pins_RAMPS_FD.h"
#elif MB(SMART_RAMPS)
    #include "module/boards/pins_SMART_RAMPS.h"
#elif MB(RAMPS4DUE)
    #include "module/boards/pins_RAMPS4DUE.h"
#elif MB(GEN6)
    #include "module/boards/pins_GEN6.h"
#elif MB(GEN6_DELUXE)
    #include "module/boards/pins_GEN6.h"
#elif MB(ALLIGATOR)
    #include "module/boards/pins_ALLIGATOR.h"
#elif MB(SANGUINOLOLU_11)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(SANGUINOLOLU_12)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(MELZI)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(STB_11)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(AZTEEG_X1)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(MELZI_1284)
    #include "module/boards/pins_SANGUINO.h"
#elif MB(AZTEEG_X3)
    #include "module/boards/pins_AZTEEG_X3.h"
#elif MB(AZTEEG_X3_PRO)
    #include "module/boards/pins_AZTEEG_X3_PRO.h"
#elif MB(ULTIMAKER)
    #include "module/boards/pins_ULTIMAKER.h"
#elif MB(MEGATRONICS)
    #include "module/boards/pins_MEGATRONICS.h"
#elif MB(MEGATRONICS_2)
    #include "module/boards/pins_MEGATRONICS_2.h"
#elif MB(MINITRONICS)
    #include "module/boards/pins_MINITRONICS.h"
#elif MB(MEGATRONICS_3)
    #include "module/boards/pins_MEGATRONICS_3.h"
#elif MB(ULTRATRONICS)
    #include "module/boards/pins_ULTRATRONICS.h"
#elif MB(ULTIMAKER_OLD)
    #include "module/boards/pins_ULTIMAKER_OLD.h"
#elif MB(ULTIMAIN_2)
    #include "module/boards/pins_ULTIMAIN_2.h"
#elif MB(3DRAG)
    #include "module/boards/pins_3DRAG.h"
#elif MB(K8200)
    #include "module/boards/pins_K8200.h"
#elif MB(TEENSYLU)
    #include "module/boards/pins_TEENSYLU.h"
#elif MB(PRINTRBOARD)
    #include "module/boards/pins_TEENSYLU.h"
#elif MB(RUMBA)
    #include "module/boards/pins_RUMBA.h"
#elif MB(BRAINWAVE)
    #include "module/boards/pins_BRAINWAVE.h"
#elif MB(SAV_MKI)
    #include "module/boards/pins_SAV_MKI.h"
#elif MB(TEENSY2)
    #include "module/boards/pins_TEENSY2.h"
#elif MB(5DPRINT)
    #include "module/boards/pins_5DPRINT.h"
#elif MB(GEN3_PLUS)
    #include "module/boards/pins_GEN3_PLUS.h"
#elif MB(OMCA_A)
    #include "module/boards/pins_OMCA_A.h"
#elif MB(OMCA)
    #include "module/boards/pins_OMCA.h"
#elif MB(LEAPFROG)
    #include "module/boards/pins_LEAPFROG.h"
#elif MB(99)
    #include "module/boards/pins_99.h"
#else
  #error Unknown MOTHERBOARD value set in Configuration.h
#endif
/****************************************************************************************/


/****************************************************************************************
******************** Available chip select pins for HW SPI ******************************
*****************************************************************************************/
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MOSI_PIN            51
  #define MISO_PIN            50
  #define SCK_PIN             52
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__)
  #define MOSI_PIN             5
  #define MISO_PIN             6
  #define SCK_PIN              7
#elif defined(__AVR_ATmega32U4__)
  #define MOSI_PIN             2
  #define MISO_PIN             3
  #define SCK_PIN              1
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
  #define MOSI_PIN            22
  #define MISO_PIN            23
  #define SCK_PIN             21
#elif defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             13
#elif defined(__AVR_ATmega1281__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             10
#elif defined (__SAM3X8E__)
  #if (SDSS == 4) || (SDSS == 10) || (SDSS == 52)|| (SDSS == 59) || (SDSS == 77)
    #if (SDSS == 4)
      #define SPI_PIN         87
      #define SPI_CHAN        1
    #elif (SDSS == 10)
      #define SPI_PIN         77
      #define SPI_CHAN        0
    #elif (SDSS == 52) 
      #define SPI_PIN         86
      #define SPI_CHAN        2
    #elif (SDSS == 59) 
      #define SPI_PIN         59
      #define SPI_CHAN        1
    #else
      #define SPI_PIN         77
      #define SPI_CHAN        0
    #endif
    #define MOSI_PIN          75
    #define MISO_PIN          74
    #define SCK_PIN           76
  #else
    #define DUE_SOFTWARE_SPI
    #define MOSI_PIN		  51
    #define MISO_PIN		  50
    #define SCK_PIN 		  52
  #endif
#endif
/****************************************************************************************/



/****************************************************************************************
********************************* END MOTHERBOARD ***************************************
****************************************************************************************/

#ifndef ORIG_E0_DIR_PIN
  #define ORIG_E0_DIR_PIN     -1
  #define ORIG_E0_ENABLE_PIN  -1
  #define ORIG_E0_STEP_PIN    -1
#endif
#ifndef ORIG_E1_DIR_PIN
  #define ORIG_E1_DIR_PIN     -1
  #define ORIG_E1_ENABLE_PIN  -1
  #define ORIG_E1_STEP_PIN    -1
#endif
#ifndef ORIG_E2_DIR_PIN
  #define ORIG_E2_DIR_PIN     -1
  #define ORIG_E2_ENABLE_PIN  -1
  #define ORIG_E2_STEP_PIN    -1
#endif
#ifndef ORIG_E3_DIR_PIN
  #define ORIG_E3_DIR_PIN     -1
  #define ORIG_E3_ENABLE_PIN  -1
  #define ORIG_E3_STEP_PIN    -1
#endif
#ifndef ORIG_E4_DIR_PIN
  #define ORIG_E4_DIR_PIN     -1
  #define ORIG_E4_ENABLE_PIN  -1
  #define ORIG_E4_STEP_PIN    -1
#endif
#ifndef ORIG_E5_DIR_PIN
  #define ORIG_E5_DIR_PIN     -1
  #define ORIG_E5_ENABLE_PIN  -1
  #define ORIG_E5_STEP_PIN    -1
#endif

#ifndef ORIG_HEATER_1_PIN
  #define ORIG_HEATER_1_PIN   -1
#endif
#ifndef ORIG_TEMP_1_PIN
  #define ORIG_TEMP_1_PIN     -1
#endif
#ifndef ORIG_HEATER_2_PIN
  #define ORIG_HEATER_2_PIN   -1
#endif
#ifndef ORIG_TEMP_2_PIN
  #define ORIG_TEMP_2_PIN     -1
#endif
#ifndef ORIG_HEATER_3_PIN
  #define ORIG_HEATER_3_PIN   -1
#endif
#ifndef ORIG_TEMP_3_PIN
  #define ORIG_TEMP_3_PIN     -1
#endif

#if ENABLED(X_STOP_PIN)
  #if X_HOME_DIR < 0
    #define ORIG_X_MIN_PIN X_STOP_PIN
    #define ORIG_X_MAX_PIN -1
  #else
    #define ORIG_X_MIN_PIN -1
    #define ORIG_X_MAX_PIN X_STOP_PIN
  #endif
#endif

#if ENABLED(Y_STOP_PIN)
  #if Y_HOME_DIR < 0
    #define ORIG_Y_MIN_PIN Y_STOP_PIN
    #define ORIG_Y_MAX_PIN -1
  #else
    #define ORIG_Y_MIN_PIN -1
    #define ORIG_Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#if ENABLED(Z_STOP_PIN)
  #if Z_HOME_DIR < 0
    #define ORIG_Z_MIN_PIN Z_STOP_PIN
    #define ORIG_Z_MAX_PIN -1
  #else
    #define ORIG_Z_MIN_PIN -1
    #define ORIG_Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifndef X_MS1_PIN
  #define X_MS1_PIN     -1
#endif
#ifndef X_MS2_PIN
  #define X_MS2_PIN     -1
#endif
#ifndef Y_MS1_PIN
  #define Y_MS1_PIN     -1
#endif
#ifndef Y_MS2_PIN
  #define Y_MS2_PIN     -1
#endif
#ifndef Z_MS1_PIN
  #define Z_MS1_PIN     -1
#endif
#ifndef Z_MS2_PIN
  #define Z_MS2_PIN     -1
#endif
#ifndef E0_MS1_PIN
  #define E0_MS1_PIN    -1
#endif
#ifndef E0_MS2_PIN
  #define E0_MS2_PIN    -1
#endif
#ifndef E1_MS1_PIN
  #define E1_MS1_PIN    -1
#endif
#ifndef E1_MS2_PIN
  #define E1_MS2_PIN    -1
#endif
#ifndef DIGIPOTSS_PIN
  #define DIGIPOTSS_PIN -1
#endif
#ifndef LCD_CONTRAST
  #define LCD_CONTRAST  -1
#endif
#ifndef Z2_MIN_PIN
  #define Z2_MIN_PIN    -1
#endif
#ifndef Z2_MAX_PIN
  #define Z2_MAX_PIN    -1
#endif

#ifndef ORIG_FAN_PIN
  #define ORIG_FAN_PIN  -1
#endif
#ifndef ORIG_FAN2_PIN
  #define ORIG_FAN2_PIN -1
#endif

#ifndef ORIG_BEEPER_PIN
  #define ORIG_BEEPER_PIN -1
#endif

/****************************************************************************************/
#include "Configuration_Pins.h"
/****************************************************************************************/


#if X_HOME_DIR > 0    // Home X to MAX
  #undef X_MIN_PIN
  #define X_MIN_PIN -1
#elif X_HOME_DIR < 0  // Home X to MIN
  #undef X_MAX_PIN
  #define X_MAX_PIN -1
#endif //X_HOME_DIR > 0

#if Y_HOME_DIR > 0    // Home Y to MAX
  #undef Y_MIN_PIN
  #define Y_MIN_PIN -1
#elif Y_HOME_DIR < 0  // Home Y to MIN
  #undef Y_MAX_PIN
  #define Y_MAX_PIN -1
#endif //Y_HOME_DIR > 0

#if Z_HOME_DIR > 0    // Home Z to MAX
  #undef Z_MIN_PIN
  #define Z_MIN_PIN -1
#elif Z_HOME_DIR < 0  // Home Z to MIN
  #undef Z_MAX_PIN
  #define Z_MAX_PIN -1
#endif //Z_HOME_DIR > 0

#if DISABLED(Z_PROBE_ENDSTOP) // Allow code to compile regardless of Z_PROBE_ENDSTOP setting.
  #define Z_PROBE_PIN -1
#endif
/****************************************************************************************/

#if ENABLED(DISABLE_XMAX_ENDSTOP)
  #undef X_MAX_PIN
  #define X_MAX_PIN -1
#endif

#if ENABLED(DISABLE_XMIN_ENDSTOP)
  #undef X_MIN_PIN 
  #define X_MIN_PIN -1
#endif

#if ENABLED(DISABLE_YMAX_ENDSTOP)
  #undef Y_MAX_PIN
  #define Y_MAX_PIN -1
#endif

#if ENABLED(DISABLE_YMIN_ENDSTOP)
  #undef Y_MIN_PIN
  #define Y_MIN_PIN -1
#endif

#if ENABLED(DISABLE_ZMAX_ENDSTOP)
  #undef Z_MAX_PIN
  #define Z_MAX_PIN -1
#endif

#if ENABLED(DISABLE_ZMIN_ENDSTOP)
  #undef Z_MIN_PIN 
  #define Z_MIN_PIN -1
#endif
/****************************************************************************************/

// List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN, analogInputToDigitalPin(TEMP_0_PIN),

#if DRIVER_EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN, analogInputToDigitalPin(TEMP_1_PIN),
#else
  #define _E1_PINS
#endif
#if DRIVER_EXTRUDERS  > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN, analogInputToDigitalPin(TEMP_2_PIN),
#else
  #define _E2_PINS
#endif
#if DRIVER_EXTRUDERS > 3
  #define _E3_PINS E3_STEP_PIN, E3_DIR_PIN, E3_ENABLE_PIN, HEATER_3_PIN, analogInputToDigitalPin(TEMP_3_PIN),
#else
  #define _E3_PINS
#endif

#define SENSITIVE_PINS { 0, 1, \
                        X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
                        Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, \
                        Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, Z_PROBE_PIN, \
                        PS_ON_PIN, HEATER_BED_PIN, FAN_PIN, \
                        _E0_PINS _E1_PINS _E2_PINS _E3_PINS \
                        analogInputToDigitalPin(TEMP_BED_PIN) \
                       }

#endif //__PINS_H
