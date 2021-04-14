//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef XNRM2_H
#define XNRM2_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[3], int ix0);

double b_xnrm2(const double x[3]);

double c_xnrm2(int n, const double x[108], int ix0);

double d_xnrm2(int n, const double x[18], int ix0);

double xnrm2(int n, const double x[9], int ix0);

double xnrm2(const double x[12]);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for xnrm2.h
//
// [EOF]
//
