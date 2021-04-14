//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lusolve.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "lusolve.h"
#include "abs.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : const double A[9]
//                const double B[3]
//                double X[3]
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
void lusolve(const double A[9], const double B[3], double X[3])
{
  double b_A[9];
  double a21;
  double maxval;
  double maxval_tmp;
  int r1;
  int r2;
  int r3;
  (void)std::copy(&A[0], &A[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval_tmp = b_abs(0.0);
  maxval = b_abs(A[0]) + maxval_tmp;
  a21 = b_abs(A[1]) + maxval_tmp;
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if ((b_abs(A[2]) + maxval_tmp) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if ((b_abs(b_A[r3 + 3]) + maxval_tmp) > (b_abs(b_A[r2 + 3]) + maxval_tmp)) {
    int rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  X[1] = B[r2] - (B[r1] * b_A[r2]);
  X[2] = (B[r3] - (B[r1] * b_A[r3])) - (X[1] * b_A[r3 + 3]);
  X[2] /= b_A[r3 + 6];
  X[0] = B[r1] - (X[2] * b_A[r1 + 6]);
  X[1] -= X[2] * b_A[r2 + 6];
  X[1] /= b_A[r2 + 3];
  X[0] -= X[1] * b_A[r1 + 3];
  X[0] /= b_A[r1];
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for lusolve.cpp
//
// [EOF]
//
