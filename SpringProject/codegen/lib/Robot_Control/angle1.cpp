//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: angle1.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "angle1.h"
#include "atan21.h"

// Function Definitions
//
// Arguments    : const creal_T x
// Return Type  : double
//
namespace Codegen {
namespace coder {
namespace internal {
namespace scalar {
double b_angle(const creal_T x)
{
  return c_atan2(x.im, x.re);
}

} // namespace scalar
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for angle1.cpp
//
// [EOF]
//
