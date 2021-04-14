//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: circshift.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "circshift.h"

// Function Definitions
//
// Arguments    : double a[5]
// Return Type  : void
//
namespace Codegen {
namespace coder {
void circshift(double a[5])
{
  double b_a[5];
  for (int i{0}; i < 5; i++) {
    b_a[i] = a[i];
  }
  b_a[0] = b_a[1];
  b_a[1] = b_a[2];
  b_a[2] = b_a[3];
  b_a[3] = b_a[4];
  b_a[4] = a[0];
  for (int i1{0}; i1 < 5; i1++) {
    a[i1] = b_a[i1];
  }
}

} // namespace coder
} // namespace Codegen

//
// File trailer for circshift.cpp
//
// [EOF]
//
