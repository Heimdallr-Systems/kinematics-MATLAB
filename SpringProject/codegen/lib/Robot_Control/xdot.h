//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdot.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef XDOT_H
#define XDOT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
double b_xdot(int n, const double x[108], int ix0, const double y[108],
              int iy0);

double c_xdot(int n, const double x[36], int ix0, const double y[36], int iy0);

double d_xdot(int n, const double x[324], int ix0, const double y[324],
              int iy0);

double xdot(int n, const double x[9], int ix0, const double y[9], int iy0);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for xdot.h
//
// [EOF]
//
