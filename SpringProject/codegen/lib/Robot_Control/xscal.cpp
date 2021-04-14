//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xscal.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "xscal.h"
#include "eml_int_forloop_overflow_check.h"

// Function Definitions
//
// Arguments    : int n
//                double a
//                double x[3]
//                int ix0
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xscal(int n, double a, double x[3], int ix0)
{
  int b;
  b = (ix0 + n) - 1;
  if ((ix0 <= b) && (b > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double a
//                double x[36]
//                int ix0
// Return Type  : void
//
void b_xscal(double a, double x[36], int ix0)
{
  int b;
  b = ix0 + 5;
  if ((ix0 <= (ix0 + 5)) && ((ix0 + 5) > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                double a
//                double x[108]
//                int ix0
// Return Type  : void
//
void c_xscal(int n, double a, double x[108], int ix0)
{
  int b;
  b = (ix0 + n) - 1;
  if ((ix0 <= b) && (b > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double a
//                double x[324]
//                int ix0
// Return Type  : void
//
void c_xscal(double a, double x[324], int ix0)
{
  int b;
  b = ix0 + 17;
  if ((ix0 <= (ix0 + 17)) && ((ix0 + 17) > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                double a
//                double x[18]
//                int ix0
// Return Type  : void
//
void d_xscal(int n, double a, double x[18], int ix0)
{
  int b;
  b = (ix0 + n) - 1;
  if ((ix0 <= b) && (b > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                double a
//                double x[9]
//                int ix0
// Return Type  : void
//
void xscal(int n, double a, double x[9], int ix0)
{
  int b;
  b = (ix0 + n) - 1;
  if ((ix0 <= b) && (b > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double a
//                double x[9]
//                int ix0
// Return Type  : void
//
void xscal(double a, double x[9], int ix0)
{
  int b;
  b = ix0 + 2;
  if ((ix0 <= (ix0 + 2)) && ((ix0 + 2) > 2147483646)) {
    check_forloop_overflow_error(true);
  }
  for (int k{ix0}; k <= b; k++) {
    x[k - 1] *= a;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for xscal.cpp
//
// [EOF]
//
