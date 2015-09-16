#include "L3G4200D.h"
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define GYR_ADDRESS (0xD2 >> 1)

// Public Methods //////////////////////////////////////////////////////////////
L3G4200D::L3G4200D(int scale,int calibration) {
	Wire.begin();

	_scale = scale;
	enableDefault();
	
	// not used yet
	calibrate(calibration);
}
// Turns on the L3G4200D's gyro and places it in normal mode.
void L3G4200D::enableDefault(void)
{
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled
	writeReg(L3G4200D_CTRL_REG1, 0x0F);

	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeReg(L3G4200D_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	writeReg(L3G4200D_CTRL_REG3, 0b00001000);

	// CTRL_REG4 controls the full-scale range, among other things:
	// default mode 2000dps -> 70 mdps/digit
	if( _scale == 250){
	  writeReg(L3G4200D_CTRL_REG4, 0b00000000);
	  speed = 8.75/(double)1000;
	}else if( _scale == 500){
	  writeReg(L3G4200D_CTRL_REG4, 0b00010000);
	  speed = 17.5/(double)1000;
	}else{
	  writeReg(L3G4200D_CTRL_REG4, 0b00110000);
	  speed = (double)70/(double)1000;
	}

	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	writeReg(L3G4200D_CTRL_REG5, 0b00000000);
}

// Writes a gyro register
void L3G4200D::writeReg(byte reg, byte value)
{
	Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

// Reads a gyro register
byte L3G4200D::readReg(byte reg)
{
	byte value;

	Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 1);
	value = Wire.read();
	Wire.endTransmission();

	return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G4200D::read()
{
	Wire.beginTransmission(GYR_ADDRESS);
	// assert the MSB of the address to get the gyro
	// to do slave-transmit subaddress updating.
	Wire.write(L3G4200D_OUT_X_L | (1 << 7));
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 6);

	while (Wire.available() < 6);

	int8_t xla = Wire.read();
	int8_t xha = Wire.read();
	int8_t yla = Wire.read();
	int8_t yha = Wire.read();
	int8_t zla = Wire.read();
	int8_t zha = Wire.read();

	g.x = xha << 8 | xla;
	g.y = yha << 8 | yla;
	g.z = zha << 8 | zla;
}

void L3G4200D::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G4200D::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G4200D::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

void L3G4200D::calibrate(int nbElt) {
	double moyAxis[3] = {0};
	double sigmaAxis[3] = {0};

	for(int k=0; k < nbElt;k++ ){
		read();

		int16_t x= (int16_t)g.x;
	  int16_t y= (int16_t)g.y;
	  int16_t z= (int16_t)g.z;

	  //Serial.println(z);

		moyAxis[0] += x;
		moyAxis[1] += y;
		moyAxis[2] += z;

		sigmaAxis[0] += x*x;
		sigmaAxis[1] += y*y;
		sigmaAxis[2] += z*z;

		delay(10);
	}
	/*Serial.println("init x y z");

	for(int i =0; i < 3 ;i++ ){
		moyAxis[i] /= nbElt;

		sigmaAxis[i] /= nbElt;
		sigmaAxis[i] -= moyAxis[i]*moyAxis[i];
		sigmaAxis[i] = sqrt(sigmaAxis[i]);

		Serial.println(i);
		Serial.println(moyAxis[i]);
		Serial.println(sigmaAxis[i]);
	}

	Serial.println("fin");*/
}

// Read 8-bit from register
uint8_t L3G4200D::readRegister8(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(GYR_ADDRESS);
  Wire.requestFrom(GYR_ADDRESS, 1);
  while(!Wire.available()) {};

	value = Wire.read();
  Wire.endTransmission();

  return value;
}

// L3G4200D Temperature sensor output change vs temperature: -1digit/degrCelsius (data representation: 2's complement).
// Value represents difference respect to a reference not specified value.
// So temperature sensor can be used to measure temperature variations: temperarture sensor isn't suitable to return absolute temperatures measures.
// If you run two sequential measures and differentiate them you can get temperature variation.
// This also means that two devices in the same temp conditions can return different outputs.
// Finally, you can use this info to compensate drifts due to temperature changes.
uint8_t L3G4200D::readTemperature(void)
{
    return readRegister8(L3G4200D_OUT_TEMP);
}
