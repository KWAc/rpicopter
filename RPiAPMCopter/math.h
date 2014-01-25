#ifndef COMMON_h
#define COMMON_h


inline
float wrap_180(float x) {
  return x < -180 ? (x + 360) : (x > 180 ? (x - 360) : x);
}

inline
float wrap_360(float x) {
  return x < 0 ? (x + 360) : (x > 360 ? (x - 360) : x);
}

float smaller(float value, float bias) {
  return value < bias ? value : bias;
}

float bigger(float value, float bias) {
  return value > bias ? value : bias;
}

float activation(float x, float force_mod = 20.f){
  float val = (180.f - smaller(abs(force_mod * x), 179.9f) ) / 180.f;
  return val / sqrt(1 + pow(val, 2));
}

#endif
