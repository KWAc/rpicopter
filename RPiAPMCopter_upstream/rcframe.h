#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;


/*
 * Abstract class:
 * Overload run() - This function is called in the main loop.
 */
class Frame {
protected:
  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;

public:
  Frame(Device *pDev, Receiver *pRecv, Exception *pExcp);
  
  // Execute the magic here!
  virtual void run() = 0;
}; 

/*
 * Implementation of an quad-copter with X-configuration
 */
class M4XFrame : public Frame {
private:
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
  M4XFrame(Device *pDev, Receiver *pRecv, Exception *pExcp);
  
  // Execute the magic here!
  void run();
};