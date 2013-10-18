#ifndef FILTERS_h
#define FILTERS_h

#include "RPiAPMCopterCommon.h"

inline
float drift_filter(float angle, float drift_vel, float &delta) {
  static long  timer        = 0;
  static float filtAngle      = 0;
  static float lastAngle     = 0;
  
  delta = 0;
  
  float dalti = angle - lastAngle;
  float time  = hal.scheduler->millis() - timer;
  float drift_const = drift_vel/1000.f*time;
  
  if(abs(dalti) > drift_const) {
    delta = dalti - drift_const;
    filtAngle += delta;
  }
  filtAngle  = wrap_180(filtAngle);  
  timer    = hal.scheduler->millis();
  
  lastAngle = angle;
  return filtAngle;
}

float sensor_fuse(float angle, 
                  float d1, float d2,
                  float w1 = 0.5, float w2 = 0.5) 
{  
  return angle + w1*d1 + w2*d2;
}

#endif
