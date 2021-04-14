//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: exp1.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "exp1.h"
#include <cmath>

// Function Definitions
//
// Arguments    : creal_T *x
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace scalar {
void c_exp(creal_T *x)
{
  if (x->im == 0.0) {
    x->re = std::exp(x->re);
    x->im = 0.0;
  } else {
    double d;
    double r;
    r = std::exp(x->re / 2.0);
    d = x->im;
    x->re = r * (r * std::cos(x->im));
    x->im = r * (r * std::sin(d));
  }
}

} // namespace scalar
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for exp1.cpp
//
// [EOF]
//
