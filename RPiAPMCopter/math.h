#ifndef COMMON_h
#define COMMON_h


inline float wrap_180(float x) {
  return x < -180 ? (x + 360) : (x > 180 ? (x - 360) : x);
}

inline float wrap_360(float x) {
  return x < 0 ? (x + 360) : (x > 360 ? (x - 360) : x);
}

inline float smaller(float value, float bias) {
  return value < bias ? value : bias;
}

inline float bigger(float value, float bias) {
  return value > bias ? value : bias;
}

inline float activation(float x, float force_mod = 20.f){
  float val = (180.f - smaller(abs(force_mod * x), 179.9f) ) / 180.f;
  return val / sqrt(1 + pow(val, 2));
}

inline uint16_t map(uint16_t x, uint16_t out_min, uint16_t out_max, uint16_t in_min = 1100, uint16_t in_max = 1900) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
