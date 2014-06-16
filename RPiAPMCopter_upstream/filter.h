#ifndef FILTER_h
#define FILTER_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>


class Functor_f {
public:
  float (*m_pfTransfer)(float, float);
  float m_fFirst;
  float m_fSecond;

  Functor_f(float (*pfFunction)(float, float), float first = 0.f, float second = 1.f) {
    m_fFirst     = first;
    m_fSecond    = second;
    m_pfTransfer = pfFunction;
  }

  float run() const {
    if(!m_pfTransfer) {
      return 0.f;
    }
    return m_pfTransfer(m_fFirst, m_fSecond);
  }

  float run(float first, float second) const {
    if(!m_pfTransfer) {
      return 0.f;
    }
    return m_pfTransfer(first, second);
  }
};

class SFilter {
public:
  // return: fSens += (fError * dT);
  static float transff_filt_f (float fSens, float fError, float dT);
  // return: fSens += (fErrorF * dTF)   + (fErrorS * dTS);
  static float transff_filt_f (float fSens, float fErrorF, float dTF, float fErrorS, float dTS);
  // return: fSens += (pFF.run() * dTF) + (pFS.run() * dTS);
  static float transff_filt_f (float fSens, const Functor_f &, float dTF, const Functor_f &,  float dTS);

  // return: fSens += fError * pfTransfer(pfVal, pfSlope) * dT;
  static float transff_filt_f (float fSens, float fError, float dT, const Functor_f &);

  /*
   * Low pass filter function prototypes
   * Method: fCurSmple * p + (fOldSmple * (1.f - p) );
   */
  static int_fast32_t low_pass_filt_l  (const long fCurr,      const long fLast,      const int p);
  static float        low_pass_filt_f  (const float fCurr,     const float fLast,     const float p);
  static Vector3f     low_pass_filt_V3f(const Vector3f &fCurr, const Vector3f &fLast, const float p);
  
  /*
   * Round 
   */
  static float round_half_f(float);
};

#endif