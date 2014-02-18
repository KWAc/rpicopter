#ifndef COMMON_h
#define COMMON_h


inline float wrap180_float(float x) {
  return x < -180.f ? (x + 360.f) : (x > 180.f ? (x - 360.f) : x);
}

inline float wrap360_float(float x) {
  return x < 0.f ? (x + 360.f) : (x > 360.f ? (x - 360.f) : x);
}

inline float smaller_float(float value, float bias) {
  return value < bias ? value : bias;
}

inline float bigger_float(float value, float bias) {
  return value > bias ? value : bias;
}

inline float delta_float(float fCurVal_Deg, float fOldVal_Deg) {
  float fVal = fCurVal_Deg - fOldVal_Deg;
  return fVal < -180.f ? ((fCurVal_Deg + 360) - fOldVal_Deg) : fVal > 180.f ? ((fCurVal_Deg - 360) - fOldVal_Deg) : fVal;
}

/*
 * Fuses two sensor values together by annealing angle_1 to angle_2
 * in every time step by a given rate value.
 */
inline float activ_float(float x, float force_mod = 20.f){
  float val = (180.f - smaller_float(abs(force_mod * x), 179.9f) ) / 180.f;
  return val / sqrt(1 + pow(val, 2));
}

inline float anneal_float(float angle_cor, float angle_fix, uint32_t time, float rate) {
  return angle_cor += wrap180_float(angle_fix-angle_cor)*((float)time/1000.f)*rate;
}

#endif
