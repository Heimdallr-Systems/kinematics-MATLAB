//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: blockedSummation.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "blockedSummation.h"

// Function Declarations
namespace Codegen {
namespace coder {
static double colMajorFlatIter(const double x[5]);

}
} // namespace Codegen

// Function Definitions
//
// Arguments    : const double x[5]
// Return Type  : double
//
namespace Codegen {
namespace coder {
static double colMajorFlatIter(const double x[5])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) + x[4];
}

//
// Arguments    : const double x[5]
// Return Type  : double
//
double blockedSummation(const double x[5])
{
  return colMajorFlatIter(x);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for blockedSummation.cpp
//
// [EOF]
//
