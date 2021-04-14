//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xscal.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef XSCAL_H
#define XSCAL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xscal(int n, double a, double x[3], int ix0);

void b_xscal(double a, double x[36], int ix0);

void c_xscal(int n, double a, double x[108], int ix0);

void c_xscal(double a, double x[324], int ix0);

void d_xscal(int n, double a, double x[18], int ix0);

void xscal(int n, double a, double x[9], int ix0);

void xscal(double a, double x[9], int ix0);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for xscal.h
//
// [EOF]
//
