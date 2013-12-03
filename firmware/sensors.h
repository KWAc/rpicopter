#ifndef SENSORS_h
#define SENSORS_h

#include <AP_Common.h>
#include <AP_Math.h>


/* 
 * Reads the current altitude in degrees and returns it as a 3D vector
 * Seems to be broken at least in my device :D I just use it as reference for sensor fusion after offset calculation
 */
 /*
inline
Vector3f get_attitude(float &roll, float &pitch, float &yaw) {
  Vector3f gyro;
  inertial.quaternion.to_euler(&gyro.x, &gyro.y, &gyro.z);

  roll  = ToDeg(gyro.x);
  pitch = ToDeg(gyro.y);
  yaw   = ToDeg(gyro.z);

  return gyro;
}
*/
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
 * Reads the current attitude in degrees and returns it as a 3D vector 
 */
inline
Vector3f get_attitude(float &roll, float &pitch, float &yaw) {
  Vector3f accel = inertial.get_accel();
  
  roll  = accel.x;
  pitch = accel.y;
  yaw   = accel.z;
  
  float r = sqrt(pow(roll, 2) + pow(pitch, 2) + pow(yaw, 2) );
  
  roll  = ToDeg(acos(accel.y/r) ) - 90.f;
  pitch = -(ToDeg(acos(accel.x/r) ) - 90.f);
  yaw   = ToDeg(acos(accel.z/r) ) - 180.f;
  
  accel.x = roll;
  accel.y = pitch;
  accel.z = yaw;
  
  return accel;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
inline
bool get_compass_heading(float &heading, 
                         float roll = 0.f, float pitch = 0.f) 
{
  static uint32_t timer = 0;
  
  if(hal.scheduler->millis() - timer >= 100) {
    compass.read();
    if (!compass.healthy) {
      hal.console->println("Compass not healthy\n");
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

float delta_angle(float fCurVal, float fOldVal) {  
  float fVal = fCurVal - fOldVal;
  
  if(fVal < -180.f) {
    fVal = (fCurVal + 360) - fOldVal;
  }
  if(fVal > 180.f) {
    fVal = (fCurVal - 360) - fOldVal;
  }
  
  return fVal;
}

/*
 * bias: Maximum countable drift in degrees
 */
inline
void measure_gyro_drift(Vector3f &drift, Vector3f &offset, int &samples, 
                        float bias = 20) 
{
  static int   counter  = -1;
  static long  timer    = 0;
  long time = hal.scheduler->millis() - timer;
    
  static float lst_rol  = 0;
  static float lst_pit  = 0;
  static float lst_yaw  = 0;
  
  static float lst_drol = 0; 
  static float lst_dpit = 0; 
  static float lst_dyaw = 0;
  
  static float sum_rol  = 0;
  static float sum_pit  = 0;
  static float sum_yaw  = 0;
  
  float rol = 0, pit = 0, yaw = 0;
  float drol = 0, dpit = 0, dyaw = 0;

  if(time >= 2000) {
    get_attitude(rol, pit, yaw);
    
    offset.x = rol;
    offset.y = pit;
    offset.z = yaw;
    
    rol = wrap_360(rol);
    pit = wrap_360(pit);
    yaw = wrap_360(yaw);

    drol = delta_angle(rol, lst_rol);
    dpit = delta_angle(pit, lst_pit);
    dyaw = delta_angle(yaw, lst_yaw);
    
    lst_rol = rol;
    lst_pit = pit;
    lst_yaw = yaw;

    if(abs(drol-lst_drol) > 0.05 || abs(dpit-lst_dpit) > 0.05 || abs(dyaw-lst_dyaw) > 0.025) {
      hal.console->printf("Gyroscope is drifting too strong. Wait for next cycle.\n");
      hal.console->printf("roll:%.3f-%.3f, pitch:%.3f-%.3f, yaw:%.3f-%.3f\n", 
                          rol, drol, 
                          pit, dpit, 
                          yaw, dyaw);
                        
      lst_drol = drol;
      lst_dpit = dpit;
      lst_dyaw = dyaw;
      
      timer = hal.scheduler->millis();
      return;
    }
    counter++;

    if(counter > 0) {
      sum_rol += drol * 1000/time;
      sum_pit += dpit * 1000/time;
      sum_yaw += dyaw * 1000/time;
      
      drift.x = sum_rol/counter;
      drift.y = sum_pit/counter;
      drift.z = sum_yaw/counter;
    } else {
      hal.console->printf("Ignore first cycle for calibration: ");
    }
    
    samples = counter;
    timer = hal.scheduler->millis();
    
    hal.console->printf("Gyroscope calibration %d - roll:%.3f-%.3f, pitch:%.3f-%.3f, yaw:%.3f-%.3f\n", 
                        samples,
                        rol, drol, 
                        pit, dpit, 
                        yaw, dyaw);
  }
}

inline
void measure_attitude_offset(Vector3f &offset) 
{     
  float rol = 0, pit = 0, yaw = 0;
  get_attitude(rol, pit, yaw);
  
  offset.x = rol;
  offset.y = pit;
  offset.z = yaw;
}

bdata get_baro() {
  static int timer = 0;

  long time = hal.scheduler->millis() - timer;
  static bdata res_data = {-1.f, -1.f, -1.f, -1.f, -1.f};
  
  if(time > 100UL) {
    timer = hal.scheduler->millis();
    barometer.read();
    uint32_t read_time = hal.scheduler->millis() - timer;
    if (!barometer.healthy) {
        hal.console->println("Barometer not healthy\n");
        return res_data;
    }
    
    res_data.pressure = barometer.get_pressure();
    res_data.altitude = barometer.get_altitude();
    res_data.temperature = barometer.get_temperature();
    res_data.climb_rate = barometer.get_climb_rate();
    res_data.pressure_samples = barometer.get_pressure_samples();

    timer = hal.scheduler->millis();
  }

  return res_data;
}

#endif
