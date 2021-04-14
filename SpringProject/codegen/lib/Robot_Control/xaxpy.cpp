//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xaxpy.h"

// Function Definitions
//
// Arguments    : int n
//                double a
//                const double x[3]
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xaxpy(int n, double a, const double x[3], int ix0, double y[9], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[108]
//                int iy0
// Return Type  : void
//
void b_xaxpy(int n, double a, int ix0, double y[108], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[108]
//                int ix0
//                double y[6]
//                int iy0
// Return Type  : void
//
void c_xaxpy(int n, double a, const double x[108], int ix0, double y[6],
             int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[36]
//                int iy0
// Return Type  : void
//
void c_xaxpy(int n, double a, int ix0, double y[36], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[6]
//                int ix0
//                double y[108]
//                int iy0
// Return Type  : void
//
void d_xaxpy(int n, double a, const double x[6], int ix0, double y[108],
             int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[324]
//                int iy0
// Return Type  : void
//
void d_xaxpy(int n, double a, int ix0, double y[324], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[9]
//                int ix0
//                double y[3]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, double a, const double x[9], int ix0, double y[3], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  if ((n >= 1) && (a != 0.0)) {
    int b;
    b = n - 1;
    for (int k{0}; k <= b; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
