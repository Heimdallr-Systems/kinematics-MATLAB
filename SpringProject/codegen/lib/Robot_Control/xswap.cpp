//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xswap.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xswap.h"

// Function Definitions
//
// Arguments    : double x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xswap(double x[9], int ix0, int iy0)
{
  double temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0 + 2];
  x[ix0 + 2] = x[iy0 + 2];
  x[iy0 + 2] = temp;
  temp = x[ix0 + 5];
  x[ix0 + 5] = x[iy0 + 5];
  x[iy0 + 5] = temp;
}

//
// Arguments    : double x[324]
//                int ix0
//                int iy0
// Return Type  : void
//
void c_xswap(double x[324], int ix0, int iy0)
{
  for (int k{0}; k < 18; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
}

//
// Arguments    : double x[36]
//                int ix0
//                int iy0
// Return Type  : void
//
void d_xswap(double x[36], int ix0, int iy0)
{
  for (int k{0}; k < 6; k++) {
    double temp;
    int i;
    int temp_tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    i = (iy0 + k) - 1;
    x[temp_tmp] = x[i];
    x[i] = temp;
  }
}

//
// Arguments    : double x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
void xswap(double x[9], int ix0, int iy0)
{
  double temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xswap.cpp
//
// [EOF]
//
