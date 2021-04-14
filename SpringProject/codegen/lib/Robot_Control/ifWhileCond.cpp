//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ifWhileCond.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "ifWhileCond.h"

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
static bool checkNoNaNs(const bool x[3]);

}
} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : const bool x[3]
// Return Type  : bool
//
namespace Codegen {
namespace coder {
namespace internal {
static bool checkNoNaNs(const bool x[3])
{
  int k;
  bool exitg1;
  bool y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

//
// Arguments    : const bool x[3]
// Return Type  : bool
//
bool ifWhileCond(const bool x[3])
{
  return checkNoNaNs(x);
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for ifWhileCond.cpp
//
// [EOF]
//
