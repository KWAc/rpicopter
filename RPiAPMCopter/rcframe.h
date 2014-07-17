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
protected:
  // Receiver channel readings
  float m_fRCRol;
  float m_fRCPit;
  float m_fRCYaw;
  float m_fRCThr;
  // Motor compensation terms (if model is tilted or battery voltage drops)
  float m_fBattComp;
  float m_fTiltComp;

  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;
  UAVNav*    m_pNavigation;
  
  // Read from receiver and copy into floats above
  void read_receiver();
  // Calculate and apply the motor compensation terms
  void calc_batt_comp();
  void calc_tilt_comp();
  void apply_comps();
  
public:
  Frame(Device *, Receiver *, Exception *, UAVNav *);

  // Execute the magic here!
  virtual void run() = 0;
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
  
protected:
  // Write to the motors
  void out();
  // Set the values to the defined minimum (see config.h)
  void clear();
  // Define speed for each motor
  void set(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t);
  // Add a certain value to the rating of the motors
  void add(int_fast16_t, int_fast16_t, int_fast16_t, int_fast16_t);

  void calc_attitude_hold();
  void calc_altitude_hold();
  void calc_gpsnavig_hold();

public:
  M4XFrame(Device *, Receiver *, Exception *, UAVNav *);

  // Execute the magic here!
  void run();
};