//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xrot.h"

// Function Definitions
//
// Arguments    : double x[324]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xrot(double x[324], int ix0, int iy0, double c, double s)
{
  for (int k{0}; k < 18; k++) {
    double b_temp_tmp;
    double d_temp_tmp;
    int c_temp_tmp;
    int temp_tmp;
    temp_tmp = (iy0 + k) - 1;
    b_temp_tmp = x[temp_tmp];
    c_temp_tmp = (ix0 + k) - 1;
    d_temp_tmp = x[c_temp_tmp];
    x[temp_tmp] = (c * b_temp_tmp) - (s * d_temp_tmp);
    x[c_temp_tmp] = (c * d_temp_tmp) + (s * b_temp_tmp);
  }
}

//
// Arguments    : double x[36]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void c_xrot(double x[36], int ix0, int iy0, double c, double s)
{
  for (int k{0}; k < 6; k++) {
    double b_temp_tmp;
    double d_temp_tmp;
    int c_temp_tmp;
    int temp_tmp;
    temp_tmp = (iy0 + k) - 1;
    b_temp_tmp = x[temp_tmp];
    c_temp_tmp = (ix0 + k) - 1;
    d_temp_tmp = x[c_temp_tmp];
    x[temp_tmp] = (c * b_temp_tmp) - (s * d_temp_tmp);
    x[c_temp_tmp] = (c * d_temp_tmp) + (s * b_temp_tmp);
  }
}

//
// Arguments    : double x[9]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(double x[9], int ix0, int iy0, double c, double s)
{
  double b_temp_tmp;
  double c_temp_tmp;
  double d_temp_tmp;
  double temp;
  double temp_tmp;
  temp_tmp = x[iy0 - 1];
  b_temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = (c * temp_tmp) - (s * b_temp_tmp);
  x[ix0 - 1] = (c * b_temp_tmp) + (s * temp_tmp);
  temp = (c * x[ix0]) + (s * x[iy0]);
  x[iy0] = (c * x[iy0]) - (s * x[ix0]);
  x[ix0] = temp;
  c_temp_tmp = x[iy0 + 1];
  d_temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = (c * c_temp_tmp) - (s * d_temp_tmp);
  x[ix0 + 1] = (c * d_temp_tmp) + (s * c_temp_tmp);
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xrot.cpp
//
// [EOF]
//
