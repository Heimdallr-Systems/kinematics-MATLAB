//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: allOrAny.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "allOrAny.h"

// Function Definitions
//
// Arguments    : const bool x[4]
// Return Type  : bool
//
namespace Codegen {
namespace coder {
namespace internal {
bool vectorAll(const bool x[4])
{
  int k;
  bool exitg1;
  bool y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for allOrAny.cpp
//
// [EOF]
//
