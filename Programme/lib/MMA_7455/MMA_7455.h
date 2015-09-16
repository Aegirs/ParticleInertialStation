// MMA_7455.h - 3 Axis Accelerometer Library
// Moritz Kemper, IAD Physical Computing Lab
// moritz.kemper@zhdk.ch
// ZHdK, 03/04/2012
// Released under Creative Commons Licence

#ifndef MMA_7455_h
#define MMA_7455_h

#include "application.h"

class MMA_7455
{
  public:
    MMA_7455(int sensitivity);

    void initSensitivity(int sensitivity);
    void calibrateOffset(float x_axis_offset, float y_axis_offset, float z_axis_offset);
    void autoCalibrateOffset(int N);

    unsigned char readAxis(char axis);
    double convertToG(int8_t data);
    double readAxisInG(char axis);
  private:
    int _sensitivity;

    unsigned char _buffer;
	  float _x_axis_offset;
	  float _y_axis_offset;
	  float _z_axis_offset;
};

#endif
