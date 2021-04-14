//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotx.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "rotx.h"
#include "cos.h"
#include "sin.h"

// Function Definitions
//
// Rotation matrix around x in radians
//
// Arguments    : double t
//                double A[9]
// Return Type  : void
//
namespace Codegen {
void rotx(double t, double A[9])
{
  double A_tmp;
  double b_A_tmp;
  A_tmp = t;
  coder::b_sin(&A_tmp);
  b_A_tmp = t;
  coder::b_cos(&b_A_tmp);
  A[0] = 1.0;
  A[3] = 0.0;
  A[6] = 0.0;
  A[1] = 0.0;
  A[4] = b_A_tmp;
  A[7] = -A_tmp;
  A[2] = 0.0;
  A[5] = A_tmp;
  A[8] = b_A_tmp;
}

} // namespace Codegen

//
// File trailer for rotx.cpp
//
// [EOF]
//
