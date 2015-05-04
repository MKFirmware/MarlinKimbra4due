/***************************************************************
*
* Externa DAC for Alligator Board
*
****************************************************************/
#include "Configuration.h"
#include "boards.h"
#include "pins.h"

#if MB(ALLIGATOR)
  #include "stepper.h"
  #include "external_dac.h"
  //#include "SPI.h"

  ExternalDac::ExternalDac() {
      return ;
  }

  void ExternalDac::begin() {
    uint8_t externalDac_buf[2] = {0x20,0x00};//all off

    pinMode (DAC_SYNC, OUTPUT);
    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    digitalWrite( SPI_EEPROM2_CS , HIGH );
    digitalWrite( SPI_FLASH_CS , HIGH );
    digitalWrite( SDSS , HIGH );
    spiBegin();

    delayMicroseconds(1U); // wait 1 microsecond
    delayMicroseconds(1U); // wait 1 microsecond
    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( DAC_SYNC , LOW );
    delay(1);
    digitalWrite( DAC_SYNC , HIGH );
    delay(1);
    digitalWrite( DAC_SYNC , LOW );

    spiSend(SPI_CHAN_DAC,externalDac_buf , 2);

    return;
  }

  void ExternalDac::setValueAll(uint8_t value) {
    uint8_t externalDac_buf[2] = {0x20,0x00};

    externalDac_buf[0] |= (value>>4); 
    externalDac_buf[1] |= (value<<4); 

    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    digitalWrite( SPI_EEPROM2_CS , HIGH );
    digitalWrite( SPI_FLASH_CS , HIGH );
    digitalWrite( SDSS , HIGH );

    delayMicroseconds(1U); // wait 1 microsecond
    delayMicroseconds(1U); // wait 1 microsecond
    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( DAC_SYNC , LOW );
    delay(1);
    digitalWrite( DAC_SYNC , HIGH );
    delay(1);
    digitalWrite( DAC_SYNC , LOW );

    spiSend(SPI_CHAN_DAC,externalDac_buf , 2);

    return;
  }

  void ExternalDac::setValue(uint8_t channel, uint8_t value) {
    if(channel>=4)
      return;
    channel = 3 - channel;

    uint8_t externalDac_buf[2] = {0x10,0x00};

    externalDac_buf[0] |= (channel<<6);
    externalDac_buf[0] |= (value>>4); 
    externalDac_buf[1] |= (value<<4); 

    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    digitalWrite( SPI_EEPROM2_CS , HIGH );
    digitalWrite( SPI_FLASH_CS , HIGH );
    digitalWrite( SDSS , HIGH );

    delayMicroseconds(1U); // wait 1 microsecond
    delayMicroseconds(1U); // wait 1 microsecond
    digitalWrite( DAC_SYNC , HIGH );
    digitalWrite( DAC_SYNC , LOW );
    delay(1);
    digitalWrite( DAC_SYNC , HIGH );
    delay(1);
    digitalWrite( DAC_SYNC , LOW );

    spiSend(SPI_CHAN_DAC,externalDac_buf , 2);

    return;
  }
#endif
