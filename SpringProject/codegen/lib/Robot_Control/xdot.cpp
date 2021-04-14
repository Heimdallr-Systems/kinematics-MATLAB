//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdot.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xdot.h"
#include "eml_int_forloop_overflow_check.h"

// Function Definitions
//
// Arguments    : int n
//                const double x[108]
//                int ix0
//                const double y[108]
//                int iy0
// Return Type  : double
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
double b_xdot(int n, const double x[108], int ix0, const double y[108], int iy0)
{
  double d;
  d = 0.0;
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error(true);
    }
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

//
// Arguments    : int n
//                const double x[36]
//                int ix0
//                const double y[36]
//                int iy0
// Return Type  : double
//
double c_xdot(int n, const double x[36], int ix0, const double y[36], int iy0)
{
  double d;
  d = 0.0;
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error(true);
    }
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

//
// Arguments    : int n
//                const double x[324]
//                int ix0
//                const double y[324]
//                int iy0
// Return Type  : double
//
double d_xdot(int n, const double x[324], int ix0, const double y[324], int iy0)
{
  double d;
  d = 0.0;
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error(true);
    }
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

//
// Arguments    : int n
//                const double x[9]
//                int ix0
//                const double y[9]
//                int iy0
// Return Type  : double
//
double xdot(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  d = 0.0;
  if (n >= 1) {
    if (n > 2147483646) {
      check_forloop_overflow_error(true);
    }
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xdot.cpp
//
// [EOF]
//
