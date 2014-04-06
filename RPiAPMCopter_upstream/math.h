#ifndef COMMON_h
#define COMMON_h

#include "config.h"


inline double progress_f(uint_fast8_t iStep, uint_fast8_t iMax) {
  return (double)iStep * 100.f / (double)iMax;
}

inline int add_flag(int flag, int mask) {
  flag |= mask;
  return flag;
}

inline int rem_flag(int flag, int mask) {
  flag &= ~mask;
  return flag;
}

inline bool chk_fset(int flag, int mask) {
  return flag & mask;
}

// Returns the sign of a float
inline short sign_l(long lVal) {
  return lVal >= 0 ? 1 : -1;
}

inline short sign_f(float fVal) {
  return fVal >= 0.f ? 1 : -1;
}

inline float pow2_f(float fVal) {
  return fVal * fVal;
}

inline long pow2_l(long lVal) {
  return lVal * lVal;
}

inline float wrap180_f(float x) {
  return x < -180 ? (x + 360.f) : (x > 180 ? (x - 360.f) : x);
}

inline float wrap360_f(float x) {
  return x < 0 ? (x + 360.f) : (x > 360 ? (x - 360.f) : x);
}

inline Vector3f wrap180_V3f(Vector3f &vec) {
  vec.x = wrap180_f(vec.x);
  vec.y = wrap180_f(vec.y);
  vec.z = wrap180_f(vec.z);
  return vec;
}

inline Vector3f wrap360_V3f(Vector3f &vec) {
  vec.x = wrap360_f(vec.x);
  vec.y = wrap360_f(vec.y);
  vec.z = wrap360_f(vec.z);
  return vec;
}

// Subtracts an from another angle (0 <= angle <= 180 && 0 >= angle >= -180)
inline float delta180_f(float fA1, float fA2) {
  fA1 = wrap180_f(fA1);
  fA2 = wrap180_f(fA2);

	float fDelta = fA1 - fA2;
	return fDelta <= -180 ? fDelta += 360.f : fDelta >= 180 ? fDelta -= 360.f : fDelta;
}

inline float smaller_f(float value, float bias) {
  return value < bias ? value : bias;
}

inline float bigger_f(float value, float bias) {
  return value > bias ? value : bias;
}

#endif
