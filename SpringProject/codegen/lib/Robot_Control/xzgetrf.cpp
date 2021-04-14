//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgetrf.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xzgetrf.h"
#include "ixamax.h"
#include "xgeru.h"
#include "xswap.h"

// Function Definitions
//
// Arguments    : double A[9]
//                int ipiv[3]
//                int *info
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace reflapack {
void xzgetrf(double A[9], int ipiv[3], int *info)
{
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  *info = 0;
  for (int j{0}; j < 2; j++) {
    int a;
    int b_tmp;
    int jp1j;
    b_tmp = j * 4;
    jp1j = b_tmp + 2;
    a = blas::ixamax(3 - j, A, b_tmp + 1);
    if (A[(b_tmp + a) - 1] != 0.0) {
      int b;
      if ((a - 1) != 0) {
        ipiv[j] = j + a;
        blas::b_xswap(A, j + 1, j + a);
      }
      b = (b_tmp - j) + 3;
      for (int i{jp1j}; i <= b; i++) {
        A[i - 1] /= A[b_tmp];
      }
    } else {
      *info = j + 1;
    }
    blas::xgeru(2 - j, 2 - j, -1.0, b_tmp + 2, b_tmp + 4, A, b_tmp + 5);
  }
  if (((*info) == 0) && (A[8] == 0.0)) {
    *info = 3;
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xzgetrf.cpp
//
// [EOF]
//
