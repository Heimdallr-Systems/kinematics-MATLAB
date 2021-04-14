//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "eye.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double b_I[9]
// Return Type  : void
//
namespace Codegen {
namespace coder {
void eye(double b_I[9])
{
  (void)std::memset(&b_I[0], 0, 9U * (sizeof(double)));
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;
}

} // namespace coder
} // namespace Codegen

//
// File trailer for eye.cpp
//
// [EOF]
//
