//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "svd1.h"
#include "xzsvdc.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : const double A[9]
//                double U[9]
//                double s[3]
//                double V[9]
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
void c_svd(const double A[9], double U[9], double s[3], double V[9])
{
  double b_A[9];
  (void)std::copy(&A[0], &A[9], &b_A[0]);
  reflapack::xzsvdc(b_A, U, s, V);
}

//
// Arguments    : const double A[108]
//                double U[36]
//                double s[6]
//                double V[324]
// Return Type  : void
//
void d_svd(const double A[108], double U[36], double s[6], double V[324])
{
  double b_A[108];
  (void)std::copy(&A[0], &A[108], &b_A[0]);
  reflapack::b_xzsvdc(b_A, U, s, V);
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for svd1.cpp
//
// [EOF]
//
