#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;
class UAVNav;


/*
 * Abstract class:
 * Basic functions for all copter-frame types and 
 * some pure virtual functions which are specific for certain frames
 */
class Frame {
private:
  // Read from receiver and copy into floats above
  void read_receiver();
  
protected:
  // Current roll, pitch, throttle and yaw readouts from the receiver module
  float m_fRCRol;
  float m_fRCPit;
  float m_fRCYaw;
  float m_fRCThr;

  // Device module pointers for high level hardware access
  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;
  UAVNav*    m_pNavigation;
  
  // Function must be overloaded for basic (and frame dependent) flight control
  virtual void servo_out()     = 0;  // Send outputs to the motors (Nr. of motors is frame dependent)
  virtual void attitude_hold() = 0;  // Calculate here, how the copter can hold attitude automatically
  virtual void altitude_hold() = 0;  // Calculate here, how the copter can hold altitude automatically
  virtual void auto_navigate() = 0;  // Here the basic Auto-GPS navigation should be implemented
  
public:
  Frame(Device *, Receiver *, Exception *, UAVNav *);

  /*
   * Updates all the necessary sensors, reads from receiver 
   * and handles exceptions.
   * Function calls: 
   * - read_receiver()
   * - calc_attitude_hold()
   * - calc_altitude_hold()
   * - calc_gpsnavig_hold()
   * - servo_out()
   */
  virtual void run();
};

/*
 * Implementation of an quad-copter with X-configuration
 */
class M4XFrame : public Frame {
private:
  uint_fast32_t m_iAltHTimer; // Altitude hold timer using barometer/sonar for GPS navigation and altitude hold mode (50 Hz)
  uint_fast32_t m_iAccZTimer; // Altitude hold timer using the accelerometer for every flight mode (if necessary). Aim: Compensation of fast g-changes along the z-axis.

private:
  // Variables holding final servo output
  int_fast16_t _FL;
  int_fast16_t _BL;
  int_fast16_t _FR;
  int_fast16_t _BR;
  
  // Motor compensation terms (if model is tilted or battery voltage drops)
  float m_fBattComp;
  float m_fTiltComp;
  
  // Calculate and apply the motor compensation terms
  void apply_motor_compens();                                       // This functions applies motor compensation terms (e.g. battery and tilt) to the output of the servos
  
  // Helper functions for regulating the servo output
  void clear();                                                     // Set the values to the defined minimum (see config.h)
  void set(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t); // Set a value for each servo
  void add(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t); // Add a certain value to the current value of the servo
  
protected:
  void servo_out();
  void attitude_hold();
  void altitude_hold();
  void auto_navigate();

public:
  M4XFrame(Device *, Receiver *, Exception *, UAVNav *);
  
  void calc_batt_comp();                                            // battery voltage drop compensation
  void calc_tilt_comp();                                            // motor compensation if model is tilted
};