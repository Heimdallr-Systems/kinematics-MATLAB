//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: error.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "error.h"
#include "Robot_Control_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
namespace Codegen {
static void b_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

}

// Function Definitions
//
// Arguments    : const char *aFcnName
//                int aLineNum
// Return Type  : void
//
namespace Codegen {
static void b_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "Detected a run-time error from a call to the function ASSERT or "
         "ERROR, but unable to report the details in standalone code. To s"
         "ee the error message, generate and execute a MEX function.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : void
// Return Type  : void
//
namespace coder {
void error()
{
  static rtRunTimeErrorInfo c_emlrtRTEI{
      24,      // lineNo
      9,       // colNo
      "error", // fName
      "D:\\Program "
      "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\lang\\error.m" // pName
  };
  b_rtErrorWithMessageID(c_emlrtRTEI.fName, c_emlrtRTEI.lineNo);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for error.cpp
//
// [EOF]
//
