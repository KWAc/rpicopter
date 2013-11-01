#ifndef COMMON_h
#define COMMON_h


inline
void flash_leds(bool on) {
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

inline
void leds_off() {
    hal.gpio->write(A_LED_PIN, LED_OFF);
    hal.gpio->write(C_LED_PIN, LED_OFF);
}

inline
void leds_on() {
    hal.gpio->write(A_LED_PIN, LED_ON);
    hal.gpio->write(C_LED_PIN, LED_ON);
}

inline
float wrap_180(float x) {
  return x < -180 ? (x + 360) : (x > 180 ? (x - 360) : x);
}

inline
float wrap_360(float x) {
  return x < 0 ? (x + 360) : (x > 360 ? (x - 360) : x);
}

float bias_value(float value, float bias) {
  return value < bias ? value : bias;
}

float activation(float x, float force_mod = 20.f){
  float val = (180.f - bias_value(abs(force_mod * x), 179.9f) ) / 180.f;
  return val / sqrt(1 + pow(val, 2));
}

#endif
