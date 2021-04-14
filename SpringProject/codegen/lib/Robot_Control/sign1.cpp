//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sign1.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "sign1.h"

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace scalar {
static void realScalarSign(double *x);

}
} // namespace internal
} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : double *x
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace scalar {
static void realScalarSign(double *x)
{
  double u;
  double y;
  u = *x;
  if (u < 0.0) {
    y = -1.0;
  } else if (u > 0.0) {
    y = 1.0;
  } else {
    y = u;
  }
  *x = y;
}

//
// Arguments    : double *x
// Return Type  : void
//
void c_sign(double *x)
{
  realScalarSign(x);
}

} // namespace scalar
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for sign1.cpp
//
// [EOF]
//
