//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef XROT_H
#define XROT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
namespace coder {
namespace internal {
namespace blas {
void b_xrot(double x[324], int ix0, int iy0, double c, double s);

void c_xrot(double x[36], int ix0, int iy0, double c, double s);

void xrot(double x[9], int ix0, int iy0, double c, double s);

} // namespace blas
} // namespace internal
} // namespace coder
} // namespace Codegen

#endif
//
// File trailer for xrot.h
//
// [EOF]
//
