#ifndef SENSORS_h
#define SENSORS_h

#include <AP_Common.h>
#include <AP_Math.h>


/* 
 * Reads the current altitude in degrees and returns it as a 3D vector 
 */
inline
Vector3f get_altitude(float &roll, float &pitch, float &yaw) {
  Vector3f gyro;
  inertial.quaternion.to_euler(&gyro.x, &gyro.y, &gyro.z);

  roll  = gyro.x = ToDeg(gyro.x);
  pitch = gyro.y = ToDeg(gyro.y);
  yaw   = gyro.z = ToDeg(gyro.z);

  return gyro;
}

/* 
 * Reads the current altitude changes in degrees and returns it as a 3D vector 
 */
inline
Vector3f get_gyroscope(float &roll, float &pitch, float &yaw) {
  Vector3f gyro = inertial.get_gyro();
  
  roll  = gyro.x = ToDeg(gyro.x);
  pitch = gyro.y = ToDeg(gyro.y);
  yaw   = gyro.z = ToDeg(gyro.z);
  
  return gyro;
}

/*
 * Return true if compass was healthy. 
 * In heading the heading of the compass is written.
 * All units in degrees
 */
inline
bool get_compass_heading(float &heading, float roll, float pitch) {
  static uint32_t timer = 0;
  
  if(hal.scheduler->millis() - timer >= 100) {
    compass.read();
    if (!compass.healthy) {
      hal.console->println("not healthy");
      return false;
    }
    Matrix3f dcm_matrix;
    dcm_matrix.from_euler(roll, pitch, 0);
    heading = compass.calculate_heading(dcm_matrix);
    heading = ToDeg(heading);
    compass.null_offsets();
    
    timer = hal.scheduler->millis();
  }
  return true;
}

/*
 * bias: Maximum countable drift in degrees
 */
inline
void measure_gyro_drift(Vector3f &drift, int &samples, float bias = 20) {
  static long  timer      = 0;
  
  static float last_rol   = 0;
  static float last_pit   = 0;
  static float last_yaw   = 0;
  
  static float sum_rol   = 0;
  static float sum_pit   = 0;
  static float sum_yaw   = 0;
  
  long time = hal.scheduler->millis() - timer;
  float rol, pit, yaw;
  float drol, dpit, dyaw;

  static int counter = 0;
  if(time >= 1000) {
    get_gyroscope(rol, pit, yaw);
    
    drol = rol - last_rol;
    dpit = pit - last_pit;
    dyaw = yaw - last_yaw;
    
    hal.console->printf("Gyroscope calibration - roll:%.3f-%.3f, pitch:%.3f-%.3f, yaw:%.3f-%.3f\n", 
                        rol, drol, pit, dpit, yaw, dyaw);
    
    // found pole
    if(abs(drol) > 180 || abs(dpit) > 180 || abs(dyaw) > 180)
      return;

    if(abs(drol) < bias && abs(dpit) < bias && abs(dyaw) < bias) {
      sum_rol += drol;
      sum_pit += dpit;
      sum_yaw += dyaw;
      
      samples = counter++;
      
      drift.x = sum_rol/counter;
      drift.y = sum_pit/counter;
      drift.z = sum_yaw/counter;
    }

    last_rol = rol;
    last_pit = pit;
    last_yaw = yaw;
    timer = hal.scheduler->millis();
  }
}

#endif