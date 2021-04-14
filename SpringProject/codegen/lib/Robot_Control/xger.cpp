//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xger.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xger.h"
#include "eml_int_forloop_overflow_check.h"

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
void xger(int m, int n, double alpha1, int ix0, int iy0, double A[9], int ia0)
{
  if (alpha1 != 0.0) {
    int jA;
    jA = ia0;
    if ((1 <= n) && (n > 2147483646)) {
      check_forloop_overflow_error(true);
    }
    for (int j{0}; j < n; j++) {
      double d;
      d = A[(iy0 + (j * 3)) - 1];
      if (d != 0.0) {
        double temp;
        int b;
        temp = d * alpha1;
        b = (m + jA) - 1;
        if ((jA <= b) && (b > 2147483646)) {
          check_forloop_overflow_error(true);
        }
        for (int ijA{jA}; ijA <= b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += 3;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xger.cpp
//
// [EOF]
//
