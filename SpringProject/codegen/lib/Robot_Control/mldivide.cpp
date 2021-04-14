//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "mldivide.h"
#include "lusolve.h"

// Function Declarations
namespace Codegen {
namespace coder {
static void mldiv(const double A[9], const double B[3], double Y[3]);

}
} // namespace Codegen

// Function Definitions
//
// Arguments    : const double A[9]
//                const double B[3]
//                double Y[3]
// Return Type  : void
//
namespace Codegen {
namespace coder {
static void mldiv(const double A[9], const double B[3], double Y[3])
{
  internal::lusolve(A, B, Y);
}

//
// Arguments    : const double A[9]
//                const double B[3]
//                double Y[3]
// Return Type  : void
//
void mldivide(const double A[9], const double B[3], double Y[3])
{
  mldiv(A, B, Y);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for mldivide.cpp
//
// [EOF]
//
