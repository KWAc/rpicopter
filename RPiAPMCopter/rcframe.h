#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;
class UAVNav;


/*
 * Abstract class:
 * Overload run() - This function is called in the main loop.
 */
class Frame {
private:
  // Motor compensation terms (if model is tilted or battery voltage drops)
  float m_fBattComp;
  float m_fTiltComp;

  // Calculate and apply the motor compensation terms
  void calc_batt_comp(); // battery voltage drop compensation
  void calc_tilt_comp(); // motor compensation if model is tilted

  // Read from receiver and copy into floats above
  void read_receiver();
  
protected:
  // Receiver channel readings
  float m_fRCRol;
  float m_fRCPit;
  float m_fRCYaw;
  float m_fRCThr;

  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;
  UAVNav*    m_pNavigation;
  
  // calls: calc_batt_comp()
  // calls: calc_tilt_comp()
  // Applies the correction terms to "m_fRCThr"
  void apply_motor_compens();
  
  // Function must be overloaded for basic (and frame dependent) flight control
  virtual void servo_out() = 0;
  virtual void calc_attitude_hold() = 0;
  virtual void calc_altitude_hold() = 0;
  virtual void calc_gpsnavig_hold() = 0;
  
public:
  Frame(Device *, Receiver *, Exception *, UAVNav *);

  // Update all the sensors, read from  receiver and handle exceptions
  // Calls: calc_attitude_hold(), calc_altitude_hold(), calc_gpsnavig_hold() and servo_out()
  virtual void run();
};

/*
 * Implementation of an quad-copter with X-configuration
 */
class M4XFrame : public Frame {
private:
  // Variables holding final motor output
  int_fast16_t _FL;
  int_fast16_t _BL;
  int_fast16_t _FR;
  int_fast16_t _BR;
  
  // Set the values to the defined minimum (see config.h)
  void clear();
  // Define speed for each motor
  void set(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t);
  // Add a certain value to the rating of the motors
  void add(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t);
  
protected:
  // Write to the motors
  void servo_out();
  void calc_attitude_hold();
  void calc_altitude_hold();
  void calc_gpsnavig_hold();

public:
  M4XFrame(Device *, Receiver *, Exception *, UAVNav *);
};