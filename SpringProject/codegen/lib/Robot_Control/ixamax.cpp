//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ixamax.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "ixamax.h"
#include "abs.h"
#include "eml_int_forloop_overflow_check.h"

// Function Definitions
//
// Arguments    : int n
//                const double x[9]
//                int ix0
// Return Type  : int
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
int ixamax(int n, const double x[9], int ix0)
{
  int idxmax;
  if (n < 1) {
    idxmax = 0;
  } else {
    idxmax = 1;
    if (n > 1) {
      double smax;
      double smax_tmp;
      smax_tmp = b_abs(0.0);
      smax = b_abs(x[ix0 - 1]) + smax_tmp;
      if (n > 2147483646) {
        check_forloop_overflow_error(true);
      }
      for (int k{2}; k <= n; k++) {
        double s;
        s = b_abs(x[(ix0 + k) - 2]) + smax_tmp;
        if (s > smax) {
          idxmax = k;
          smax = s;
        }
      }
    }
  }
  return idxmax;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for ixamax.cpp
//
// [EOF]
//
