// **************************************************************************
//
// Description: Fast IO functions for Arduino Due
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef	_FASTIO_SAM_H
#define	_FASTIO_SAM_H

// --------------------------------------------------------------------------
// magic I/O routines
// now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
// --------------------------------------------------------------------------

/// Read a pin
#define     READ(IO)        digitalRead(IO)
/// Write to a pin
#define     WRITE(IO, v)    digitalWrite(IO, v)

/// set pin as input
#define     SET_INPUT(IO)   pinMode (IO, INPUT)
/// set pin as output
#define     SET_OUTPUT(IO)  pinMode (IO, OUTPUT)

/// toggle a pin	
#define    TOGGLE(IO)      WRITE(IO, !READ(IO))

/// set pin as input_pullup
#define     SET_PULLUP(IO)  pinMode (IO, INPUT_PULLUP)

// Write doesn't work for pullups
#define PULLUP(IO,v) {pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

/// check if pin is an input
#define    GET_INPUT(IO)
/// check if pin is an output
#define    GET_OUTPUT(IO)

/// check if pin is an timer
#define GET_TIMER(IO)

// Shorthand
#define OUT_WRITE(IO, v) { SET_OUTPUT(IO); WRITE(IO, v); }

#endif	//_FASTIO_SAM_H
