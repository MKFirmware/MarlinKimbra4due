// BitMap for splashscreen
// Generated with: http://www.digole.com/tools/PicturetoC_Hex_converter.php
// Please note that using the high-res version takes 402Bytes of PROGMEM.
#define START_BMPHIGH

#if ENABLED(SHOW_BOOTSCREEN)
  #if ENABLED(START_BMPHIGH)
    #define START_BMPWIDTH      112
    #define START_BMPHEIGHT      38
    #define START_BMPBYTEWIDTH   14
    #define START_BMPBYTES      532 // START_BMPWIDTH * START_BMPHEIGHT / 8

    const unsigned char start_bmp[START_BMPBYTES] PROGMEM = {
      0x3,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0xf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0x1e,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x78,0x38,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1c,0x70,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xe,0x60,0x0,0x0,0x0,0x0,0x30,0x0,0x0,0x0,0x0,0x1,0x0,0x0,0x6,0xe0,0x1f,0x7c,0x0,0x0,0x33,0x0,0x0,0x0,0x0,0x1,0x80,0x3,0x87,0xc0,0x3f,0xfe,0x0,0x0,0x33,0x0,0x0,0x0,0x0,0x1,0xc0,0x2,0x83,
      0xc0,0x73,0xce,0x0,0x0,0x30,0x0,0x0,0x0,0x0,0x3,0xc0,0x3,0x83,0xc0,0xf3,0xce,0x3c,0xf,0x33,0xf,0x0,0x0,0x0,0x3,0xc0,0x2,0x83,0xc0,0xe3,0x8e,0x7e,0x1f,0xb3,0x1f,0x80,0x0,0x0,0x1f,0xe0,0x0,0x3,0xc0,0xe3,0x8e,0xe7,0x39,0xb3,0x39,0xc0,0x0,0x0,0xff,0xfc,0x3,0x83,0xc0,0xe3,0x8e,0xc3,0x31,0xb3,0x38,0xc0,0x0,0x0,0x3f,0xff,0x2,0x3,0xc0,0xe3,0x8e,0xc3,0x30,0x33,0x38,0xc0,0x0,0x0,0x7,0xf0,0x2,0x3,0xc0,0xe3,0x8e,0xe3,0x30,0x33,0x38,0xc0,0x0,0x1,0x7,0xf0,0x3,0x83,0xc0,0xe3,0x8e,0xfb,0xf0,0x3f,0xf8,0xc0,0x0,0x3,0x83,0xf0,0x0,0x3,
      0xc0,0x63,0x8e,0x7b,0xf0,0x1f,0xf8,0xc0,0x0,0x7,0xc1,0xf8,0x0,0x3,0xc0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x3,0x9f,0xfe,0x0,0x3,0xc0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xff,0xff,0x80,0x43,0xe0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x7f,0xff,0xf8,0xc3,0xf0,0x0,0x0,0x0,0x0,0x0,0x3,0x0,0x0,0x0,0x33,0xff,0xff,0xc3,0xf8,0x0,0x0,0x30,0xf0,0x0,0x3,0x0,0x0,0x0,0x19,0xff,0xff,0x83,0xfc,0x0,0x0,0x71,0xe6,0x0,0x3,0x0,0x0,0x0,0xc,0x7f,0xff,0x3,0xfe,0x0,0x0,0x73,0xc6,0x0,0x3,0x0,0x0,0x0,0x6,0x7f,0xff,0x3,
      0xff,0x0,0x0,0x77,0x80,0x0,0x3,0x0,0x0,0x0,0x3,0x3f,0xfe,0x3,0xff,0x80,0x0,0x7f,0x6,0x7f,0xe3,0xf0,0xf0,0xf0,0x1,0x9f,0xfc,0x3,0xff,0xc0,0x0,0x7e,0x6,0xff,0xf3,0xf9,0xf9,0xf8,0x0,0xcf,0xf8,0x3,0xff,0xe0,0x0,0x7c,0x6,0xe7,0x33,0x9b,0x9b,0x9c,0x0,0x67,0xf0,0x3,0xff,0xf0,0x0,0x7e,0x6,0xe3,0x1b,0x1b,0x1b,0xc,0x0,0x33,0xc0,0x3,0xff,0xf8,0x0,0x7f,0x6,0xe3,0x1b,0x1b,0x3,0xc,0x0,0x19,0x0,0x3,0xff,0xfc,0x0,0x73,0x86,0xe3,0x1b,0x3b,0x3,0x8c,0x0,0xc,0x0,0x3,0xff,0xfe,0x0,0x71,0xc6,0xe3,0x1b,0xf3,0x3,0xef,0x0,0x6,0x0,0x7,
      0xff,0xff,0x0,0x30,0xe6,0x63,0x19,0xe3,0x1,0xef,0x0,0x2,0x0,0x6,0xff,0xff,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xe,0xff,0xff,0xc0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1c,0xff,0xff,0xe0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x78,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0
    };
  #else
    #define START_BMPWIDTH      56
    #define START_BMPHEIGHT     19
    #define START_BMPBYTEWIDTH  7
    #define START_BMPBYTES      133 // START_BMPWIDTH * START_BMPHEIGHT / 8

    const unsigned char start_bmp[START_BMPBYTES] PROGMEM = {
      0x1f,0xff,0xff,0xff,0xff,0xff,0xfc,0x30,0x0,0x0,0x0,0x0,0x0,0x6,0x60,0x0,0x0,0x0,0x0,0x0,0x3,0xc3,0x60,0x5,0x0,0x0,0x8,0x1,0x84,0x90,0x4,0x0,0x0,0x18,0x1,0x84,0x97,0x75,0x70,0x0,0x3c,0x1,0x84,0x95,0x45,0x50,0x0,0x7f,0x1,0x84,0x95,0x45,0x50,0x0,0x1c,0x1,
      0x84,0x97,0xc5,0x50,0x1,0x8e,0x1,0x80,0x0,0x0,0x0,0x0,0xff,0x81,0xc0,0x0,0x0,0x10,0x0,0x5f,0xf9,0xe0,0x2,0xd0,0x10,0x0,0x27,0xf1,0xf0,0x3,0x80,0x10,0x0,0x17,0xf1,0xf8,0x3,0x17,0xde,0xee,0xb,0xe1,0xfc,0x3,0x15,0x52,0x8a,0x5,0x81,0xfe,0x3,0x95,0x52,0x8a,0x2,0x1,
      0xff,0x2,0xd5,0x5e,0x8f,0x1,0x1,0xff,0x80,0x0,0x0,0x0,0x0,0x3,0xff,0xff,0xff,0xff,0xff,0xff,0xfe
    };
  #endif
#endif

// Here comes a compile-time operation to match the extruder symbols
// on the info screen to the set number of hotends in configuration.h
//
// When only one extruder is selected, the "1" on the symbol will not
// be displayed.

#if HOTENDS == 1
  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen0_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x0C, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0E, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x0F, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x0F, 0xA0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x07, 0xA0,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x5E, 0x07, 0xA0,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x5F, 0x0F, 0xA0,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x4F, 0x0F, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x47, 0x0E, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x63, 0x0C, 0x60,
    0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };

  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen1_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xF8, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xF8, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xF0, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x01, 0xA0,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x58, 0x01, 0xA0,
    0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x40, 0x60, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x40, 0xF0, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x41, 0xF8, 0x20,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x61, 0xF8, 0x60,
    0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };
#elif HOTENDS == 2
  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen0_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x0C, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0E, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x0F, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x0F, 0xA0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x07, 0xA0,
    0x7F, 0x80, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0xFB, 0xC0, 0x00, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xF3, 0xC0, 0x00, 0x76, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xEB, 0xC0, 0x00, 0x7E, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0x7B, 0x80, 0x00, 0x3D, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x5E, 0x07, 0xA0,
    0x7B, 0x80, 0x00, 0x3B, 0xC0, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x5F, 0x0F, 0xA0,
    0xFB, 0xC0, 0x00, 0x77, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x4F, 0x0F, 0x20,
    0xFB, 0xC0, 0x00, 0x70, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x47, 0x0E, 0x20,
    0xFF, 0xC0, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x63, 0x0C, 0x60,
    0x3F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };

  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen1_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xF8, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xF8, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xF0, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x01, 0xA0,
    0x7F, 0x80, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0xFB, 0xC0, 0x00, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xF3, 0xC0, 0x00, 0x76, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xEB, 0xC0, 0x00, 0x7E, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0x7B, 0x80, 0x00, 0x3D, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x58, 0x01, 0xA0,
    0x7B, 0x80, 0x00, 0x3B, 0xC0, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x40, 0x60, 0x20,
    0xFB, 0xC0, 0x00, 0x77, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x04, 0x10, 0x00, 0x40, 0xF0, 0x20,
    0xFB, 0xC0, 0x00, 0x70, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x82, 0x08, 0x00, 0x41, 0xF8, 0x20,
    0xFF, 0xC0, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x41, 0x04, 0x00, 0x61, 0xF8, 0x60,
    0x3F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };
#else
  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen0_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x0C, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0E, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x0F, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x0F, 0xA0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x07, 0xA0,
    0x7F, 0x80, 0x00, 0x3F, 0xC0, 0x00, 0x3F, 0xC0, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0xFB, 0xC0, 0x00, 0x79, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xF3, 0xC0, 0x00, 0x76, 0xE0, 0x00, 0x76, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xEB, 0xC0, 0x00, 0x7E, 0xE0, 0x00, 0x7E, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0x7B, 0x80, 0x00, 0x3D, 0xC0, 0x00, 0x39, 0xC0, 0x00, 0x82, 0x08, 0x00, 0x5E, 0x07, 0xA0,
    0x7B, 0x80, 0x00, 0x3B, 0xC0, 0x00, 0x3E, 0xC0, 0x01, 0x04, 0x10, 0x00, 0x5F, 0x0F, 0xA0,
    0xFB, 0xC0, 0x00, 0x77, 0xE0, 0x00, 0x76, 0xE0, 0x01, 0x04, 0x10, 0x00, 0x4F, 0x0F, 0x20,
    0xFB, 0xC0, 0x00, 0x70, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x82, 0x08, 0x00, 0x47, 0x0E, 0x20,
    0xFF, 0xC0, 0x00, 0x7F, 0xE0, 0x00, 0x7F, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x63, 0x0C, 0x60,
    0x3F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x0F, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };

  #define STATUS_SCREENWIDTH     115 //Width in pixels
  #define STATUS_SCREENHEIGHT     19 //Height in pixels
  #define STATUS_SCREENBYTEWIDTH  15 //Width in bytes
  const unsigned char status_screen1_bmp[] PROGMEM = { //AVR-GCC, WinAVR
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xF8, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xF8, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xF0, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x01, 0xA0,
    0x7F, 0x80, 0x00, 0x3F, 0xC0, 0x00, 0x3F, 0xC0, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0xFB, 0xC0, 0x00, 0x79, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xF3, 0xC0, 0x00, 0x76, 0xE0, 0x00, 0x76, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x5E, 0xF7, 0xA0,
    0xEB, 0xC0, 0x00, 0x7E, 0xE0, 0x00, 0x7E, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x5C, 0x63, 0xA0,
    0x7B, 0x80, 0x00, 0x3D, 0xC0, 0x00, 0x39, 0xC0, 0x00, 0x82, 0x08, 0x00, 0x58, 0x01, 0xA0,
    0x7B, 0x80, 0x00, 0x3B, 0xC0, 0x00, 0x3E, 0xC0, 0x01, 0x04, 0x10, 0x00, 0x40, 0x60, 0x20,
    0xFB, 0xC0, 0x00, 0x77, 0xE0, 0x00, 0x76, 0xE0, 0x01, 0x04, 0x10, 0x00, 0x40, 0xF0, 0x20,
    0xFB, 0xC0, 0x00, 0x70, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x82, 0x08, 0x00, 0x41, 0xF8, 0x20,
    0xFF, 0xC0, 0x00, 0x7F, 0xE0, 0x00, 0x7F, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x61, 0xF8, 0x60,
    0x3F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x0F, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
  };
#endif // Hotends
