//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgetrf.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xgetrf.h"
#include "xzgetrf.h"

// Function Definitions
//
// Arguments    : double A[9]
//                int ipiv[3]
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace lapack {
void xgetrf(double A[9], int ipiv[3])
{
  int info;
  reflapack::xzgetrf(A, ipiv, &info);
}

} // namespace lapack
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xgetrf.cpp
//
// [EOF]
//
