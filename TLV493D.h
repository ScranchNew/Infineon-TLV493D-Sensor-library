/***************************************************
  Arduino library for the Infineo TLV493D Magnetic hall effect sensor
Based on a code example from
 *   Mark J. Hughes
 *   for AllAboutCircuits.com
 *	https://www.allaboutcircuits.com/technical-articles/tutorial-and-overview-of-infineons-3d-magnetic-2go-kit/

  The TLV 493 D sensor detects a 3D magnetic field
****************************************************/

#ifndef TLV493D_H
#define TLV493D_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <Wire.h>

bool i2cSelect(uint8_t i);          // This function is only required, if you use the Adafruit TCA9548A 1-to-8 I2C Multiplexer
// it selects the chosen I2C-port on the multiplexer

class field_data {                  // a class to hold the raw data of one sensor read
public:
  field_data();
  field_data(int16_t x_d, int16_t y_d, int16_t z_d, int16_t t_d);

  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int16_t t = 0;
};


class TLV493D {				    // This class is used to read out one TLV 493 Sensor

public:
  TLV493D();

  byte        start(byte address_, byte mult_ = 8); // starts the sensor at the given address and sets it to Low Power Mode
  
  byte        getAddress();         // returns I2C-Address of this sensor
  byte        getMult();            // returns Multiplexer port of this sensor
						    // only needed, if you use the Adafruit TCA9548A 1-to-8 I2C Multiplexer

  
// read functions: use read() or readData() to get new Sensordata

  bool        read();               // Updates data for all read functions

  field_data  readData();           // Updates data for all read functions and also returns it

// output functions: These functions just return the values that were updated in the last read() or readData() call.

  double      readTemp();           // returns last Temperature reading  in °C
  int16_t     readTempRaw();        // returns last raw Temperature reading
  double      readField_x();        // returns last X-Field reading in mT
  int16_t     readField_xRaw();     // returns last raw X-Field reading
  double      readField_y();        // returns last Y-Field reading in mT
  int16_t     readField_yRaw();     // returns last raw Y-Field reading
  double      readField_z();        // returns last Z-Field reading in mT
  int16_t     readField_zRaw();     // returns last raw Z-Field reading

  double      readAngle_x();        // returns Angle of magnetic field around the axis
  double      readAngle_y();        
  double      readAngle_z();        
  
private:
  byte        address  = 0;         // I2C address
  byte        mult     = 0;         // Multiplexer port


  field_data    data;               // Data of last read
  
  byte        rbuffer[10];          // store data from sensor read registers
  byte        wbuffer[4];           // store data for sensor write registers.
  byte        error = 0;            // stores if a read error happened on one of the sensors
  byte        debugcounter = 0;     // variable for debug counter

  int16_t     decodeX(int a, int b);  // Decode Routines
  int16_t     decodeY(int a, int b);
  int16_t     decodeZ(int a, int b);
  int16_t     decodeT(int a, int b);
};

#endif //A1335_H
