//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "norm.h"
#include "xnrm2.h"

// Function Declarations
namespace Codegen {
namespace coder {
static double b_genpnorm(const double x[3]);

static double genpnorm(const double x[12]);

} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : const double x[3]
// Return Type  : double
//
namespace Codegen {
namespace coder {
static double b_genpnorm(const double x[3])
{
  return internal::blas::b_xnrm2(x);
}

//
// Arguments    : const double x[12]
// Return Type  : double
//
static double genpnorm(const double x[12])
{
  return internal::blas::xnrm2(x);
}

//
// Arguments    : const double x[12]
// Return Type  : double
//
double b_norm(const double x[12])
{
  return genpnorm(x);
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
double c_norm(const double x[3])
{
  return b_genpnorm(x);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for norm.cpp
//
// [EOF]
//
