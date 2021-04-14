//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: roty.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "roty.h"
#include "cos.h"
#include "sin.h"

// Function Definitions
//
// Rotation matrix around y in radians
//
// Arguments    : double t
//                double A[9]
// Return Type  : void
//
namespace Codegen {
void roty(double t, double A[9])
{
  double A_tmp;
  double b_A_tmp;
  A_tmp = t;
  coder::b_sin(&A_tmp);
  b_A_tmp = t;
  coder::b_cos(&b_A_tmp);
  A[0] = b_A_tmp;
  A[3] = 0.0;
  A[6] = A_tmp;
  A[1] = 0.0;
  A[4] = 1.0;
  A[7] = 0.0;
  A[2] = -A_tmp;
  A[5] = 0.0;
  A[8] = b_A_tmp;
}

} // namespace Codegen

//
// File trailer for roty.cpp
//
// [EOF]
//
