//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef SVD1_H
#define SVD1_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
void c_svd(const double A[9], double U[9], double s[3], double V[9]);

void d_svd(const double A[108], double U[36], double s[6], double V[324]);

} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for svd1.h
//
// [EOF]
//
