//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "svd.h"
#include "svd1.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[108]
//                double U[36]
//                double S[108]
//                double V[324]
// Return Type  : void
//
namespace Codegen {
namespace coder {
void b_svd(const double A[108], double U[36], double S[108], double V[324])
{
  double s[6];
  internal::d_svd(A, U, s, V);
  (void)std::memset(&S[0], 0, 108U * (sizeof(double)));
  for (int k{0}; k < 6; k++) {
    S[k + (6 * k)] = s[k];
  }
}

//
// Arguments    : const double A[9]
//                double U[9]
//                double S[9]
//                double V[9]
// Return Type  : void
//
void svd(const double A[9], double U[9], double S[9], double V[9])
{
  double s[3];
  internal::c_svd(A, U, s, V);
  (void)std::memset(&S[0], 0, 9U * (sizeof(double)));
  S[0] = s[0];
  S[4] = s[1];
  S[8] = s[2];
}

} // namespace coder
} // namespace Codegen

//
// File trailer for svd.cpp
//
// [EOF]
//
