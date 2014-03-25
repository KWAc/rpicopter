#ifndef SFUSE_h
#define SFUSE_h

class Device;

/*
 * The Device class contains only basic access functions
 * The only exception is the built in accelerometer & gyrometer sensor fusion (bcause it's handy)
 * Here further sensor fusion functions are implemented
 */
float climbrate_cms(Device *, bool &);       // Atm fused barometer and accelerometer readings
int_fast32_t altitude_cm(Device*, bool &);   // Atm fused barometer and GPS readings

///////////////////////////////////////////////////////////
// Change the sign in contrast to device class readout,
// because the motors should spin higher if the copter is falling 
// ==> return must be positive in this case
///////////////////////////////////////////////////////////
// Scaled from 0.fX g - 1.fX g (or higher)
// Positive when accelerating down, negative when accelerating up
///////////////////////////////////////////////////////////
Vector3f accel_g(Device*, bool &);

#endif
