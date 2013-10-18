#ifndef FILTERS_h
#define FILTERS_h

#include "RPiAPMCopterCommon.h"


inline
float drift_filter(float &filt_angle,
                   float curr_angle, float last_angle,
                   float drift_vel, float time) 
{
  float delta = 0;
  float dalti = curr_angle - last_angle;
  float drift_const = drift_vel/1000.f*time;
  
  if(abs(dalti) > drift_const) {
    delta = dalti - drift_const;
    filt_angle += delta;
  }
  filt_angle = wrap_180(filt_angle);  
  last_angle = curr_angle;
  return delta;
}

float sensor_fuse(float gyro, float compass, float time, float rate = 0.025) {  
  return gyro += wrap_180(compass-gyro)*(time/1000)*rate;
}

#endif
