#ifndef SFUSE_h
#define SFUSE_h

class Device;

/*
 * The Device class contains only basic access functions
 * The only exception is the built in accelerometer & gyrometer sensor fusion (bcause it's handy)
 * Here further sensor fusion functions are implemented
 */
float climbrate_ms(Device *, bool &);       // Atm fused barometer and accelerometer readings
float altitude_m(Device*, bool &);  // Atm fused barometer and GPS readings
float altitude_cm(Device*, bool &);  // Atm fused barometer and GPS readings

#endif
