//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzsvdc.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef XZSVDC_H
#define XZSVDC_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace reflapack {
void b_xzsvdc(double A[108], double U[36], double S[6], double V[324]);

void xzsvdc(double A[9], double U[9], double S[3], double V[9]);

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for xzsvdc.h
//
// [EOF]
//
