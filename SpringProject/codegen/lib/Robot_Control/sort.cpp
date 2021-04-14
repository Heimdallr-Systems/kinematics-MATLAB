//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sort.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "sort.h"
#include "sortIdx.h"

// Function Definitions
//
// Arguments    : double x[4]
//                int idx[4]
// Return Type  : void
//
namespace Codegen {
namespace coder {
namespace internal {
void sort(double x[4], int idx[4])
{
  sortIdx(x, idx);
}

} // namespace internal
} // namespace coder
} // namespace Codegen

//
// File trailer for sort.cpp
//
// [EOF]
//
