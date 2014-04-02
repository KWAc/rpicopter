#include "filter.h"


float SFilter::transff_filt_f(float fSens, float fError, float dT) {
  fSens += fError * dT;
  return fSens;
}

float SFilter::transff_filt_f(float fSens, float fError, float dT, const Functor_f &functor) {
  fSens += fError * functor.run() * dT;
  return fSens;
}

/*
 * Low pass filter
 */
int_fast32_t SFilter::low_pass_filt_l(const long fCurSmple, const long fOldSmple, const int p) {
  const int q = 100 - p;
  return (fCurSmple * p + fOldSmple * q) / 100;
}

float SFilter::low_pass_filt_f(const float fCurSmple, const float fOldSmple, const float p) {
  return fCurSmple * p + (fOldSmple * (1.f - p) );
}

Vector3f SFilter::low_pass_filt_V3f(const Vector3f &fCurSmple, const Vector3f &fOldSmple, const float p) {
  return fCurSmple * p + (fOldSmple * (1.f - p) );
}