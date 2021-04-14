//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: findTrue4Elem.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "findTrue4Elem.h"

// Function Definitions
//
// Searces through a four element array and returns the indices to the
//  elements that are true. Returned array is always 4 elements even if less
//  than four elements are true. This allows fixed length code generation.
//
// Arguments    : const bool arrayToSearch[4]
//                unsigned char indices[4]
// Return Type  : void
//
namespace Codegen {
void findTrue4Elem(const bool arrayToSearch[4], unsigned char indices[4])
{
  unsigned char elToAppend;
  indices[0] = 0U;
  indices[1] = 0U;
  indices[2] = 0U;
  indices[3] = 0U;
  elToAppend = 1U;
  if (arrayToSearch[0]) {
    indices[0] = 1U;
    elToAppend = 2U;
  }
  if (arrayToSearch[1]) {
    indices[(static_cast<int>(elToAppend)) - 1] = 2U;
    elToAppend = static_cast<unsigned char>(
        static_cast<int>((static_cast<int>(elToAppend)) + 1));
  }
  if (arrayToSearch[2]) {
    indices[(static_cast<int>(elToAppend)) - 1] = 3U;
    elToAppend = static_cast<unsigned char>(
        static_cast<int>((static_cast<int>(elToAppend)) + 1));
  }
  if (arrayToSearch[3]) {
    indices[(static_cast<int>(elToAppend)) - 1] = 4U;
  }
}

} // namespace Codegen

//
// File trailer for findTrue4Elem.cpp
//
// [EOF]
//
