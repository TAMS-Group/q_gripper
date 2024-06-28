/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#ifndef HX711_LITE_h
#define HX711_LITE_h

#include "Arduino.h"


class HX711_LITE
{
  private:
    byte PD_SCK;  // Power Down and Serial Clock Input Pin
    byte DOUT;    // Serial Data Output Pin
    byte GAIN_CLOCK_CYCLES;    // extra clock cycles (1..3) to select input and amplification 
    long OFFSET = 0;  // used for tare weight
    float SCALE = 1.0;  // used to return weight in grams, kg, ounces, whatever


  public:
    const static long NOT_READY = 0x55000000;

    HX711_LITE();

    virtual ~HX711_LITE();

    // Initialize library with data output pin, clock input pin and gain factor.
    // Channel selection is made by passing the appropriate gain:
    // - With a gain factor of 64 or 128, channel A is selected
    // - With a gain factor of 32, channel B is selected
    // The library default is "128" (Channel A).
    void begin(byte dout, byte pd_sck, byte gain = 128);

    // Check if HX711 is ready
    // from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
    // input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
    bool is_ready();

    // set the gain factor; takes effect only after a call to read()
    // channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
    // depending on the parameter, the channel is also set to either A or B
    void set_gain(byte gain = 128);

    // waits for the chip to be ready and returns a reading
    long read();

    double read_scaled_value(); // subtract offset, multiply with scale

    // set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
    void set_scale(float scale = 1.f);

    // get the current SCALE
    float get_scale();

    // set OFFSET, the value that's subtracted from the actual reading (tare weight)
    void set_offset(long offset = 0);

    // get the current OFFSET
    long get_offset();

    // puts the chip into power down mode
    void power_down();

    // wakes up the chip after power down mode
    void power_up();
};

#endif /* HX711_LITE_h */
