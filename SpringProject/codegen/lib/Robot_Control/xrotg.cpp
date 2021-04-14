//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrotg.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xrotg.h"
#include "abs.h"
#include "sqrt.h"

// Function Definitions
//
// Arguments    : double *a
//                double *b
//                double *c
//                double *s
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void xrotg(double *a, double *b, double *c, double *s)
{
  double absa;
  double absb;
  double b_c;
  double b_s;
  double d;
  double roe;
  double scale;
  roe = *b;
  absa = b_abs(*a);
  absb = b_abs(*b);
  if (absa > absb) {
    roe = *a;
  }
  scale = absa + absb;
  if (scale == 0.0) {
    b_s = 0.0;
    b_c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    double ads;
    double bds;
    double r;
    ads = absa / scale;
    bds = absb / scale;
    d = (ads * ads) + (bds * bds);
    b_sqrt(&d);
    r = scale * d;
    if (roe < 0.0) {
      r = -r;
    }
    b_c = (*a) / r;
    b_s = (*b) / r;
    if (absa > absb) {
      *b = b_s;
    } else if (b_c != 0.0) {
      *b = 1.0 / b_c;
    } else {
      *b = 1.0;
    }
    *a = r;
  }
  *c = b_c;
  *s = b_s;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xrotg.cpp
//
// [EOF]
//
