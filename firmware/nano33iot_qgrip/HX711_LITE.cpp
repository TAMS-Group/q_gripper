/**
 * HX711_LITE library for "fast" Arduino (SAMD, ESP32, ...)
 * based on HX711 library for Arduino https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/

#include <Arduino.h>
#include "HX711_LITE.h"


// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU  


HX711_LITE::HX711_LITE() {
}


HX711_LITE::~HX711_LITE() {
}


void HX711_LITE::begin(byte dout, byte pd_sck, byte gain) {
	PD_SCK = pd_sck;
	DOUT = dout;

	pinMode(PD_SCK, OUTPUT);
	pinMode(DOUT, INPUT_PULLUP);

	set_gain(gain);
}


/** 
 * HX711 sets DATA to HIGH while converting, LOW indicates ready.
 * Read twice with 1 usec in between to avoid timing issues.
 */
bool HX711_LITE::is_ready() {
	bool b1 = (digitalRead(DOUT) == LOW);
  delayMicroseconds(1);
	bool b2 = (digitalRead(DOUT) == LOW);
  return b1 && b2; 
}


/**
 * set the number of extra clock cycles after reading a value.
 * For the HX711 chip, this selects the input (A,B) and opamp gain.
 * Legal input values are 128 (input A, gain 128, 1 cycle), 
 * 64 (input A, gain 64, 3 cycles), and 32 (input B, gain 32, 2 cycles).
 */
void HX711_LITE::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN_CLOCK_CYCLES = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN_CLOCK_CYCLES = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN_CLOCK_CYCLES = 2;
			break;
	}
}


long HX711_LITE::read() 
{
  // we don't want to wait here, but we still check is_ready().
  // If NOT ready, we return the special value HX711_NOT_READY.
  if (!is_ready()) return HX711_LITE::NOT_READY;
  \

	// Define structures for reading data into.
	unsigned long value = 0;
  unsigned long    v2 = 0;
  uint8_t v = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;


	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.

  unsigned long t0 = micros();
  // disable interrupts
  noInterrupts();

  // read loop, 24 bits, MSB first, at least 0.1 usec and typical 1 usec 
  // for every SCK state
  //
  for( unsigned int i = 0; i < 24; i++ ) {

    digitalWrite( PD_SCK, HIGH );
    delayMicroseconds(1);

    v = digitalRead( DOUT );
    value = (value << 1) | (v & 0x000000ff);

    delayMicroseconds(1);
    v = digitalRead( DOUT ); // read again
    v2 = (v2 << 1) | (v & 0x000000ff);

    digitalWrite( PD_SCK, LOW );
    delayMicroseconds(1);
  }

  if (v2 != value) {
    Serial.print( "HX711_LITE.read: " ); Serial.print( value ); Serial.print( " vs " ); Serial.print( v2 ); Serial.print( " <<< " );
  }

  // fill in upper 8-bits to generate proper signed output value
  if (value & 0x00800000) {
    value = value | 0xff000000;
  }

  // extra clock cycles to select the input (A,B) and gain (128,64,32).
  // Ensure that the clock line is low at the end of this method.
  //
  for( unsigned int i = 0; i < GAIN_CLOCK_CYCLES; i++ ) {
		digitalWrite( PD_SCK, HIGH );
		delayMicroseconds( 1 );
		digitalWrite( PD_SCK, LOW );
		delayMicroseconds( 1 );
  }

	// enable interrupts again.
	interrupts();

  return value;
}


double HX711_LITE::read_scaled_value() {
  long value = read();
  return (value - OFFSET) * SCALE;
}


void HX711_LITE::set_scale(float scale) {
	SCALE = scale;
}

float HX711_LITE::get_scale() {
	return SCALE;
}

void HX711_LITE::set_offset(long offset) {
	OFFSET = offset;
}

long HX711_LITE::get_offset() {
	return OFFSET;
}


/**
 * the HX711 goes to power-down mode after 60 usec of logical HIGH on PD_SCK line.
 */
void HX711_LITE::power_down() {
	digitalWrite(PD_SCK, LOW);
  delayMicroseconds( 1 );
	digitalWrite(PD_SCK, HIGH);
  delayMicroseconds( 1 );
}


void HX711_LITE::power_up() {
	digitalWrite(PD_SCK, LOW);
  delayMicroseconds( 1 );
}

// end HX711_LITE.cpp
