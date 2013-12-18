#ifndef SETUP_h
#define SETUP_h


/* 
 * Find the offset from total equilibrium.
 * Because the vehicle is never totally horizontally, 
 * we will measure the discrepancy to compensate unequal motor thrust at start.
 */
Vector3f attitude_calibration() {
  Vector3f offset;
  float samples_acc[ATTITUDE_SAMPLE_CNT];
  float samples_avg = 0;
  float samples_dev = 0;
  
  // initially switch on LEDs
  leds_on(); bool led = true;
  
  while(true) {
    samples_avg = 0;
    samples_dev = 0;
    
    GYRO_ROL_OFFS = 0;
    GYRO_PIT_OFFS = 0;
    GYRO_YAW_OFFS = 0;
  
    // Take 10 samples in less than one second
    for(int i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
      inertial.update();
      measure_attitude_offset(offset);

      hal.console->printf("Gyroscope calibration - Offsets are roll:%f, pitch:%f, yaw:%f.\n", 
                          offset.x, offset.y, offset.z);

      GYRO_ROL_OFFS += offset.x / (float)ATTITUDE_SAMPLE_CNT;
      GYRO_PIT_OFFS += offset.y / (float)ATTITUDE_SAMPLE_CNT;
      GYRO_YAW_OFFS += offset.z / (float)ATTITUDE_SAMPLE_CNT;
      
      // Check whether the data set is useable
      float cur_sample = sqrt(pow(offset.x, 2) + pow(offset.y, 2) + pow(offset.z, 2) );
      samples_acc[i] = cur_sample;
      samples_avg += cur_sample / ATTITUDE_SAMPLE_CNT;
    
      flash_leds(led); led = !led;  // let LEDs blink
      hal.scheduler->delay(50);     //Wait 50ms
    }
    
    // Calc standard deviation
    for(int i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
    samples_dev += pow(samples_acc[i] - samples_avg, 2) / (float)ATTITUDE_SAMPLE_CNT;
    }
    samples_dev = sqrt(samples_dev);
    // If std dev is low: exit loop
    if(samples_dev < samples_avg / 16.f) 
      break;
  }
	
  // Save offsets
  offset.x = GYRO_ROL_OFFS;
  offset.y = GYRO_PIT_OFFS;
  offset.z = GYRO_YAW_OFFS;
    
  leds_off();   // switch off leds
  hal.console->printf("Gyroscope calibrated - Offsets are roll:%f, pitch:%f, yaw:%f.\nAverage euclidian distance:%f, standard deviation:%f\n", 
                      GYRO_ROL_OFFS, GYRO_PIT_OFFS, GYRO_YAW_OFFS, samples_avg, samples_dev);
  return offset;
}

void init_baro() {
  barometer.init();
  barometer.calibrate();
}

void init_pids() {
  PIDS[PID_PIT_RATE].kP(0.65);
  PIDS[PID_PIT_RATE].kI(0.30);
  PIDS[PID_PIT_RATE].imax(50);

  PIDS[PID_ROL_RATE].kP(0.65);
  PIDS[PID_ROL_RATE].kI(0.30);
  PIDS[PID_ROL_RATE].imax(50);

  PIDS[PID_YAW_RATE].kP(1.25);
  PIDS[PID_YAW_RATE].kI(0.25);
  PIDS[PID_YAW_RATE].imax(50);

  PIDS[PID_PIT_STAB].kP(5.5);
  PIDS[PID_ROL_STAB].kP(5.5);
  PIDS[PID_YAW_STAB].kP(5.5);
}

void init_compass() {
  if(!compass.init() ) {
    COMPASS_INITIALIZED = 0;
    hal.console->printf("Init compass failed!\n");
  }

  compass.accumulate();
  compass.motor_compensation_type(1);                              // throttle
  compass.set_offsets(0, 0, 0);                                    // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.f) );                            // set local difference between magnetic north and true north

  hal.console->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
  case AP_COMPASS_TYPE_HIL:
    hal.console->printf("HIL\n");
    break;
  case AP_COMPASS_TYPE_HMC5843:
    hal.console->println("HMC5843\n");
    break;
  case AP_COMPASS_TYPE_HMC5883L:
    hal.console->printf("HMC5883L\n");
    break;
  case AP_COMPASS_TYPE_PX4:
    hal.console->printf("PX4\n");
    break;
  default:
    hal.console->printf("unknown\n");
    break;
  }
  COMPASS_INITIALIZED = 1;
}

void init_inertial() {
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  inertial.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
}

void init_gps() {
  gps.init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
}

void init_batterymon() {
  // initialise the battery monitor
  battery.init();
  battery.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
}

#endif
