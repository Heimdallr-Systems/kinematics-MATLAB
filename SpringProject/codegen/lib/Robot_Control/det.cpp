//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: det.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "det.h"
#include "xgetrf.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : const double x[9]
// Return Type  : double
//
namespace Codegen {
namespace coder {
double det(const double x[9])
{
  double b_x[9];
  double y;
  int ipiv[3];
  bool isodd;
  (void)std::copy(&x[0], &x[9], &b_x[0]);
  internal::lapack::xgetrf(b_x, ipiv);
  isodd = (ipiv[0] > 1);
  y = (b_x[0] * b_x[4]) * b_x[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }
  if (isodd) {
    y = -y;
  }
  return y;
}

} // namespace coder
} // namespace Codegen

//
// File trailer for det.cpp
//
// [EOF]
//
