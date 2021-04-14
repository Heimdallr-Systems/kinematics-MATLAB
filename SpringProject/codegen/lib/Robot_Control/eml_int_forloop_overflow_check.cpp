//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_int_forloop_overflow_check.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "eml_int_forloop_overflow_check.h"
#include "Robot_Control_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
namespace Codegen {
static void rtErrorWithMessageID(const char *b, const char *aFcnName,
                                 int aLineNum);

}

// Function Definitions
//
// Arguments    : const char *b
//                const char *aFcnName
//                int aLineNum
// Return Type  : void
//
namespace Codegen {
static void rtErrorWithMessageID(const char *b, const char *aFcnName,
                                 int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "The loop variable of class ") << b)
      << " might overflow on the last iteration of the for loop. This could "
         "lead to an infinite loop.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : bool overflow
// Return Type  : void
//
namespace coder {
void check_forloop_overflow_error(bool overflow)
{
  static rtRunTimeErrorInfo c_emlrtRTEI{
      88,                             // lineNo
      9,                              // colNo
      "check_forloop_overflow_error", // fName
      "D:\\Program "
      "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
      "overflow_check.m" // pName
  };
  if (overflow) {
    rtErrorWithMessageID("int32", c_emlrtRTEI.fName, c_emlrtRTEI.lineNo);
  }
}

} // namespace coder
} // namespace Codegen

//
// File trailer for eml_int_forloop_overflow_check.cpp
//
// [EOF]
//
