#ifndef FILTERS_h
#define FILTERS_h

#include "RPiAPMCopterCommon.h"


/*
 * Return the drift corrected delta of the current and the last angle
 */
inline
float drift_filter(float curr_angle, float last_angle,
                   float drift_vel, float time) 
{
  float delta = 0;
  float dalti = curr_angle - last_angle;
  float drift_const = drift_vel/1000.f*time;
  
//  if(abs(dalti) > drift_const) {
    delta = dalti - drift_const;
//  }
  last_angle = curr_angle;
  return delta;
}

/*
 * Fuses two sensor values together by annealing angle_1 to angle_2 
 * in every time step by a given rate value.
 */
float sensor_fuse(float angle_cor, float angle_fix, 
                  float time, float rate = 0.025) 
{  
  return angle_cor += wrap_180(angle_fix-angle_cor)*(time/1000)*rate;
}

#endif
