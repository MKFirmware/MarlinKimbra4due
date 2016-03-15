#ifndef CONFIGURATION_VERSION_H
  #define CONFIGURATION_VERSION_H

  #define FIRMWARE_NAME "MK4due"
  #define SHORT_BUILD_VERSION "4.2.8_dev"
  #define BUILD_VERSION FIRMWARE_NAME "_" SHORT_BUILD_VERSION
  #define STRING_DISTRIBUTION_DATE __DATE__ " " __TIME__    // build date and time
  // It might also be appropriate to define a location where additional information can be found
  #define FIRMWARE_URL  "https://github.com/MagoKimbra/MarlinKimbra4due"
#endif
