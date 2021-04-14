//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: combineVectorElements.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "combineVectorElements.h"
#include "blockedSummation.h"
#include "sumprod.h"

// Function Definitions
//
// Arguments    : const double x[6]
// Return Type  : double
//
namespace Codegen {
namespace coder {
double b_combineVectorElements(const double x[6])
{
  double y;
  y = x[0];
  for (int k{0}; k < 5; k++) {
    y *= x[k + 1];
  }
  return y;
}

//
// Arguments    : const double x[5]
// Return Type  : double
//
double c_combineVectorElements(const double x[5])
{
  return blockedSummation(x);
}

//
// Arguments    : const bool x[4]
// Return Type  : int
//
int combineVectorElements(const bool x[4])
{
  int y;
  y = static_cast<int>(x[0]);
  nnzfun(&y, static_cast<int>(x[1]));
  nnzfun(&y, static_cast<int>(x[2]));
  nnzfun(&y, static_cast<int>(x[3]));
  return y;
}

} // namespace coder
} // namespace Codegen

//
// File trailer for combineVectorElements.cpp
//
// [EOF]
//
