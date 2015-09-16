// MMA_7455.cpp - 3 Axis Accelerometer Library
// Moritz Kemper, IAD Physical Computing Lab
// moritz.kemper@zhdk.ch
// ZHdK, 03/04/2012
// Released under Creative Commons Licence

#include "MMA_7455.h"

#define MMA_7455_ADDRESS 0x1D //I2C Adsress for the sensor
#define MMA_7455_MODE_CONTROLL 0x16 //Call the sensors Mode Control

#define MMA_7455_2G_MODE 0x05 //Set Sensitivity to 2g
#define MMA_7455_4G_MODE 0x09 //Set Sensitivity to 4g
#define MMA_7455_8G_MODE 0x01 //Set Sensitivity to 8g

#define X_OUT 0x06 //Register for reading the X-Axis
#define Y_OUT 0x07 //Register for reading the Y-Axis
#define Z_OUT 0x08 //Register for reading the Z-Axis

MMA_7455::MMA_7455(int sensitivity)
{
  Wire.begin();

  //default
  _sensitivity = sensitivity;
  _x_axis_offset = 0;
  _y_axis_offset = 0;
  _z_axis_offset = 0;

  // Set the sensitivity you want to use
  // 2 = 2g, 4 = 4g, 8 = 8g
  initSensitivity(sensitivity);

  // Calibrate the Offset, that values corespond in
  // à tester
  autoCalibrateOffset(100);
  //accelero.calibrateOffset(9, 23, -7);
}

void MMA_7455::initSensitivity(int sensitivity)
{
  _sensitivity = sensitivity;

  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.write(MMA_7455_MODE_CONTROLL);
  if(sensitivity == 2)
  {
    Wire.write(MMA_7455_2G_MODE);  //Set Sensitivity to 2g Detection
  }
  if(sensitivity == 4)
  {
    Wire.write(MMA_7455_4G_MODE);  //Set Sensitivity to 4g Detection
  }
  if(sensitivity == 8)
  {
    Wire.write(MMA_7455_8G_MODE);  //Set Sensitivity to 8g Detection
  }
  Wire.endTransmission();
  delay(1000);
}

void MMA_7455::calibrateOffset(float x_axis_offset, float y_axis_offset, float z_axis_offset)
{
  _x_axis_offset = x_axis_offset;
  _y_axis_offset = y_axis_offset;
  _z_axis_offset = z_axis_offset;
}

void MMA_7455::autoCalibrateOffset(int N)
{
  double x_axis_offset = 0, y_axis_offset = 0, z_axis_offset = 0;
  for(int i = 0 ; i < N ; i++ ) {
    x_axis_offset += (int8_t)readAxis('x');
    y_axis_offset += (int8_t)readAxis('y');
    z_axis_offset += (int8_t)readAxis('z');

    delay(10);
  }

  x_axis_offset /= N;
  y_axis_offset /= N;
  z_axis_offset /= N;

  _x_axis_offset = -1 * x_axis_offset;
  _y_axis_offset = -1 * y_axis_offset;
  _z_axis_offset = (2*63/_sensitivity) - z_axis_offset;

}

unsigned char MMA_7455::readAxis(char axis)
{
  Wire.beginTransmission(MMA_7455_ADDRESS);
  if(axis == 'x' || axis == 'X')
  {
    Wire.write(X_OUT);
  }
  if(axis == 'y' || axis == 'Y')
  {
    Wire.write(Y_OUT);
  }
  if(axis == 'z' || axis == 'Z')
  {
    Wire.write(Z_OUT);
  }

  Wire.endTransmission();
  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.requestFrom(MMA_7455_ADDRESS, 1);

  if(Wire.available())
  {
    _buffer = Wire.read();
  }
  if(axis == 'x' || axis == 'X')
  {
    _buffer = _buffer + _x_axis_offset;
  }
  if(axis == 'y' || axis == 'Y')
  {
    _buffer = _buffer + _y_axis_offset;
  }
  if(axis == 'z' || axis == 'Z')
  {
    _buffer = _buffer + _z_axis_offset;
  }

  return _buffer;
}

double MMA_7455::convertToG(int8_t data) {
  int rang = (2*63)/_sensitivity;
  return (double)data/(double)rang;
}

double MMA_7455::readAxisInG(char axis) {
  char val; //Variables for the values from the sensor

  val = readAxis(axis); //Read out the 'axis' Axis

  int8_t x = (int8_t)val;
  double xG = convertToG(x);

  return xG;
}
