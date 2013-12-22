#ifndef SENSORS_h
#define SENSORS_h

#include <AP_Common.h>
#include <AP_Math.h>


/* 
 * Reads the current altitude changes from the gyroscope in degrees and returns it as a 3D vector 
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
 * Reads the current attitude from the accelerometer in degrees and returns it as a 3D vector 
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
    if (!compass.healthy() ) {
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

BaroData get_baro() {
  static int timer = 0;

  long time = hal.scheduler->millis() - timer;
  static BaroData res_data = {-1.f, -1.f, -1.f, -1.f, -1.f};
  
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

GPSData get_gps() {
  static GPSData gps_data = {-1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f};
  
  gps.update();
  if(gps.new_data) {   
    if(gps.fix) {
      gps_data.latitude    = gps.latitude;
      gps_data.longitude   = gps.longitude;
      gps_data.altitude_m  = (float)gps.altitude_cm / 100.0;
      
      gps_data.gspeed_ms   = (float)gps.ground_speed_cm / 100.0;
      gps_data.espeed_ms   = gps.velocity_east();
      gps_data.nspeed_ms   = gps.velocity_north();
      gps_data.dspeed_ms   = gps.velocity_down();
      
      // The fucking avr_g++ does NOT support dynamic C++ with class like obects in structs declared as static :(
      // Hate this primitivity
      gps_data.heading_x   = gps.velocity_vector().x;
      gps_data.heading_y   = gps.velocity_vector().y;
      gps_data.heading_z   = gps.velocity_vector().z;
      
      gps_data.gcourse_cd  = (int)gps.ground_course_cd / 100;
      gps_data.status_fix  = gps.fix;
      gps_data.satelites   = gps.num_sats;
      gps_data.time_week   = gps.time_week;
      gps_data.time_week_s = gps.time_week_ms / 1000.0;
    } else {
      // Dunno atm
    }
    gps.new_data = false;
  }
  return gps_data;
}

BattData get_battery() {
  static BattData res_data = {-1.f, -1.f, -1.f};

  static int timer = 0;
  long time = hal.scheduler->millis() - timer;
  
  if(time > 100UL) {
    battery.read();

    res_data.voltage_V    = battery.voltage();
    res_data.current_A    = battery.current_amps();
    res_data.consumpt_mAh = battery.current_total_mah();
    
    timer = hal.scheduler->millis();
  }

  return res_data;
}

/*
Function from a LiPo charging chart:
4,20 V  100%
4,13 V	90%
4,06 V	80%
3,99 V	70%
3,92 V	60%
3,85 V	50%
3,78 V	40%
3,71 V	30%
3,64 V	20%
3,57 V	10%
3,50 V  0%

Return: 0 - 1 (0%-100%) if in voltage range of this table :D
*/
float residual_LiPoCapac(float voltage_V, unsigned int num_cells) {
  return  1.4286 * (voltage_V / (float)num_cells) - 5.f;
}

#endif
