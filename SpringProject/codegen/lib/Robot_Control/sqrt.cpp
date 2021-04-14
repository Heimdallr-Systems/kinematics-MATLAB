//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sqrt.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "sqrt.h"
#include "Robot_Control_types.h"
#include "sqrt1.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
namespace Codegen {
static void b_rtErrorWithMessageID(const char *b, const char *aFcnName,
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
static void b_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "Domain error. To compute complex results from real x, use \'")
   << b)
      << "(complex(x))\'.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : double *x
// Return Type  : void
//
namespace coder {
void b_sqrt(double *x)
{
  static rtRunTimeErrorInfo c_emlrtRTEI{
      13,     // lineNo
      9,      // colNo
      "sqrt", // fName
      "D:\\Program "
      "Files\\MATLAB\\R2021a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m" // pName
  };
  if ((*x) < 0.0) {
    b_rtErrorWithMessageID("sqrt", c_emlrtRTEI.fName, c_emlrtRTEI.lineNo);
  }
  internal::scalar::c_sqrt(x);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for sqrt.cpp
//
// [EOF]
//
