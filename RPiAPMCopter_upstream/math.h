#ifndef COMMON_h
#define COMMON_h

#include "config.h"


inline double progress_f(uint_fast8_t iStep, uint_fast8_t iMax) {
  return (double)iStep*100.f/(double)iMax;
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
inline float sign_f(float fVal) {
  return fVal >= 0.f ? 1.f : -1.f;
}

inline long sign_l(long lVal) {
  return lVal >= 0 ? 1 : -1;
}

inline float pow2_f(float fVal) {
  return fVal * fVal;
}

inline long pow2_l(long lVal) {
  return lVal * lVal;
}

/*
 * Return: Length of a vector "vec"
 */
inline float vecl3f_f(const Vector3f &vec) {
  return sqrt(pow2_f(vec.x) + pow2_f(vec.y) + pow2_f(vec.z) );
}

inline float wrap180_f(float x) {
  return x < -180.f ? (x + 360.f) : (x > 180.f ? (x - 360.f) : x);
}

inline float wrap360_f(float x) {
  return x < 0.f ? (x + 360.f) : (x > 360.f ? (x - 360.f) : x);
}
/*
 * This function changes the function parameter
 * Used this for performance reasons
 */
inline Vector3f wrap180_V3f(Vector3f &vec) {
  vec.x = wrap180_f(vec.x);
  vec.y = wrap180_f(vec.y);
  vec.z = wrap180_f(vec.z);
  return vec;
}

/*
 * This function changes the function parameter
 * Used this for performance reasons
 */
inline Vector3f wrap360_V3f(Vector3f &vec) {
  vec.x = wrap360_f(vec.x);
  vec.y = wrap360_f(vec.y);
  vec.z = wrap360_f(vec.z);
  return vec;
}

inline float smaller_f(float value, float bias) {
  return value < bias ? value : bias;
}

inline float bigger_f(float value, float bias) {
  return value > bias ? value : bias;
}

inline float anneal_f(float &fSens, const float fErr, const float fSigm_dT) {
  fSens += fErr * fSigm_dT;
  return fSens;
}

/*
 * Low pass filter
 */
inline int_fast32_t 
low_pass_filter_l(const int_fast32_t fCurSmple, const int_fast32_t fOldSmple, const int_fast16_t p) 
{
  const int_fast16_t q = 100 - p;
  return (fCurSmple * p + fOldSmple * q) / 100;
}

inline float 
low_pass_filter_f(const float fCurSmple, const float fOldSmple, const float p) 
{
  return fCurSmple * p + (fOldSmple * (1.f - p) );
}

inline Vector3f 
low_pass_filter_V3f(const Vector3f &fCurSmple, const Vector3f &fOldSmple, const float p) 
{
  return fCurSmple * p + (fOldSmple * (1.f - p) );
}

#endif
