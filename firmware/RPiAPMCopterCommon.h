#ifndef COMMON_h
#define COMMON_h

inline
float wrap_180(float x) {
  return x < -180 ? (x + 360) : (x > 180 ? (x - 360) : x);
}

/*inline
float abs(float val) {
  return val < 0.f ? -val : val;
}*/

#endif
