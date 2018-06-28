/***************************************************
  Arduino library for the Infineo TLV493D Magnetic hall effect sensor
Based on a code example from
 *   Mark J. Hughes
 *   for AllAboutCircuits.com
 *	https://www.allaboutcircuits.com/technical-articles/tutorial-and-overview-of-infineons-3d-magnetic-2go-kit/

  The TLV 493 D sensor detects a 3D magnetic field
****************************************************/

#include "TLV493D.h"
#include <math.h>

const byte TCAADDR = 0x70;
 
bool i2cSelect(uint8_t i) {
  if (i > 7){
    return false;
  }
  
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  
  delayMicroseconds(50);
  return true;
}

const byte  addr_1 = 0x1F; // default address of magnetic sensor with SDA on HIGH: 0x1F
const byte  addr_2 = 0x5E; // default address of magnetic sensor with SDA on LOW:  0x5E


//--- Begin Write Registers ---//
/*  
 *  Mode 1 is the second write register
 *  Mode1_Int   Bxxxxx1xx  Interrupt Enable "1" / Disable "0"
 *  Mode1_Fast  Bxxxxxx1x  Fast Mode Enable "1" / Disable "0" must be 0 for power down
 *  Mode1_Low   Bxxxxxxx1  Low Power Mode Enable "1" / Disable "0"
 *  
 *  Mode 2 is the fourth write register
 *  Mode2_T     B1xxxxxxx  Temperature Measurement Enable "1" / Disable "0"
 *  Mode2_LP    Bx1xxxxxx  LP Period "1" = 12ms / "0"=100ms
 *  Mode2_PT    Bxx1xxxxx  Parity test Enable "1" / Disable "0"
 *  
 */

//     Example settings for Ultra-Low Power, Low Power, Fast Mode, and Power Down.
//                        Reg 1      Reg 2      Reg 3      Reg 4           
const byte ulpm[] = { B00000000, B00000001, B00000000, B00000000 }; // ultra low power mode
const byte lpm[]  = { B00000000, B00000001, B00000000, B01000000 }; // low power mode
const byte fm[]   = { B00000000, B00000010, B00000000, B00000000 }; // fast mode (unsupported)
const byte pd[]   = { B00000000, B00000001, B00000000, B00000000 }; // power down mode.

field_data::field_data(){
  
}

field_data::field_data(int16_t x_d, int16_t y_d, int16_t z_d, int16_t t_d){
  x = x_d;
  y = y_d;
  z = z_d;
  t = t_d;
}

TLV493D::TLV493D(){
  
}


byte   TLV493D::getAddress(){
  return address;
}

byte   TLV493D::getMult(){
  return mult;
}

byte      TLV493D::start(byte address_, byte mult_){
  address = address_;
  mult = mult_;
  i2cSelect(mult);
  Wire.requestFrom(address, byte(10));
    for(int i=0; i < 10; i++){
      if(Wire.available()) {
        rbuffer[i] = Wire.read();
      }
    }
    // Write Register 0H is non configurable.  Set all bits to 0
    wbuffer[0] = B00000000;
    // Read Register 7H 6:3 -> Write Register 1H 6:3
    wbuffer[1] = rbuffer[7] & B01111000;
    
    // Read Register 8H 7:0 -> Write Register 2H 7:0
    wbuffer[2] = rbuffer[8];
    
    // Read Register 9H 4:0 -> Write Register 3H 4:0 (Mod2)
    wbuffer[3] = rbuffer[9] & B00001111;
      
    // Set Power Mode (ulpm, lpm, fm, pd)
    for(int i=0; i < 4; i++){
      wbuffer[i] |= lpm[i];
    }
    
    Wire.beginTransmission(address);        
    for(int i=0; i < 4; i++){ 
      Wire.write(wbuffer[i]);               
      }
    Wire.endTransmission();
    delayMicroseconds(100);
}

bool        TLV493D::read(){
  i2cSelect(mult);
  Wire.requestFrom(address, byte(6));
  for(int i=0; i < 6; i++){
    if(Wire.available()){
      rbuffer[i] = Wire.read();
    }
  }
  
  if(rbuffer[3] & B00000011 != 0){ // If bits are not 0, TLV is still reading Bx, By, Bz, or T
    error = error | 0x1;
    //Serial.println("Reading error!");
    return false;
  }
  
  data.x = decodeX(rbuffer[0],rbuffer[4]);
  data.y = decodeY(rbuffer[1],rbuffer[4]);
  data.z = decodeZ(rbuffer[2],rbuffer[5]);
  data.t = decodeT(rbuffer[3],rbuffer[6]);
  return true;
}

field_data  TLV493D::readData(){
  read();
  return data;
}

double      TLV493D::readTemp(){
  return data.t*double(1.1);
}

int16_t     TLV493D::readTempRaw(){
  return data.t;
}

double      TLV493D::readField_x(){
  return data.x * double(0.098);
  
}

int16_t     TLV493D::readField_xRaw(){
  return data.x;
  
}

double      TLV493D::readField_y(){
  return data.y * double(0.098);
  
}

int16_t     TLV493D::readField_yRaw(){
  return data.y;
  
}

double      TLV493D::readField_z(){
  return data.z * double(0.098);
  
}

int16_t     TLV493D::readField_zRaw(){
  return data.z;
  
}

double      TLV493D::readAngle_x() {       // returns Angle of magnetic field around the axis in ° [-90° to 270°]
  double result;
  result = atan(double(data.z)/double(data.y));
  result *= 180.0/M_PI;
  if (data.y < 0) {
    result += 180.0;
  }
  return result;
}

double      TLV493D::readAngle_y() {
  double result;
  result = atan(double(data.x)/double(data.z));
  result *= 180.0/M_PI;
  if (data.z < 0) {
    result += 180.0;
  }
  return result;
}
      
double      TLV493D::readAngle_z(){
  double result;
  result = atan(double(data.y)/double(data.x));
  result *= 180.0/M_PI;
  if (data.x < 0) {
    result += 180.0;
  }
  return result;
  
}

int16_t     TLV493D::decodeX(int a, int b){
/* Shift all bits of register 0 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0:3 shift in as zero.
 * Determine which of bits 4:7 of register 4 are high, shift them to the right four places -- remask in case
 * they shift in as something other than 0.  bitRead and bitWrite would be a bit more elegant in next version
 * of code.
 */
  int16_t ans = ( a << 4 ) | (((b & B11110000) >> 4) & B00001111);
  if (ans & 0x800) {
    ans &= 0x7FF;
    ans -= 2048;
  } // Interpret bit 12 as +/-
  return ans;
}

int16_t     TLV493D::decodeY(int a, int b){
/* Shift all bits of register 1 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 shift in as zero.
 * Determine which of the first four bits of register 4 are true.  Add to previous answer.
 */

  int16_t ans = (a << 4) | (b & B00001111);
  if (ans & 0x800) {
    ans &= 0x7FF;
    ans -= 2048;
  } // Interpret bit 12 as +/-
  return ans;
}

int16_t     TLV493D::decodeZ(int a, int b){
/* Shift all bits of register 2 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 are zero.
 * Determine which of the first four bits of register 5 are true.  Add to previous answer.
 */
  int16_t ans = (a << 4) | (b & B00001111);
  if (ans & 0x800) {
    ans &= 0x7FF;
    ans -= 2048;
  } // Interpret bit 12 as +/-
  return ans;
}

int16_t     TLV493D::decodeT(int a, int b){
/* Determine which of the last 4 bits of register 3 are true.  Shift all bits of register 3 to the left 
 * 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 are zero.
 * Determine which of the first four bits of register 6 are true.  Add to previous answer.
 */
  int16_t ans;
  a &= B11110000;
  ans = (a << 4) | b;
  if (ans & 0x800) {
    ans &= 0x7FF;
    ans -= 2048;
  } // Interpret bit 12 as +/-
  return ans;
}
