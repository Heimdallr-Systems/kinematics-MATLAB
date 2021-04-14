//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeru.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xgeru.h"
#include "xger.h"

// Function Definitions
//
// Arguments    : int m
//                int n
//                double alpha1
//                int ix0
//                int iy0
//                double A[9]
//                int ia0
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void xgeru(int m, int n, double alpha1, int ix0, int iy0, double A[9], int ia0)
{
  xger(m, n, alpha1, ix0, iy0, A, ia0);
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xgeru.cpp
//
// [EOF]
//
