//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "diag.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double v[3]
//                double d[9]
// Return Type  : void
//
namespace Codegen {
namespace coder {
void diag(const double v[3], double d[9])
{
  (void)std::memset(&d[0], 0, 9U * (sizeof(double)));
  d[0] = v[0];
  d[4] = v[1];
  d[8] = v[2];
}

} // namespace coder
} // namespace Codegen

//
// File trailer for diag.cpp
//
// [EOF]
//
