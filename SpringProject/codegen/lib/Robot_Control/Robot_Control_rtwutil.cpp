//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Robot_Control_rtwutil.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Definitions
//
// Arguments    : unsigned int b
// Return Type  : unsigned short
//
namespace Codegen {
unsigned short _u16_u32_(unsigned int b)
{
  unsigned short a;
  a = static_cast<unsigned short>(b);
  if ((static_cast<unsigned int>(a)) != b) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

//
// Arguments    : const char *aFcnName
//                int aLineNum
// Return Type  : void
//
void c_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Domain error. To compute complex results, make at least one "
               "input complex, e.g. \'power(complex(a),b)\'.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : int aIndexValue
//                int aLoBound
//                int aHiBound
//                const rtBoundsCheckInfo *aInfo
// Return Type  : void
//
void rtDynamicBoundsError(int aIndexValue, int aLoBound, int aHiBound,
                          const rtBoundsCheckInfo *aInfo)
{
  std::stringstream b_outStream;
  std::stringstream outStream;
  if (aLoBound == 0) {
    aIndexValue++;
    aLoBound = 1;
    aHiBound++;
  }
  if (rtIsNullOrEmptyString(aInfo->aName)) {
    ((((((b_outStream << "Index exceeds array dimensions.  Index value ")
         << aIndexValue)
        << " exceeds valid range [")
       << aLoBound)
      << "-")
     << aHiBound)
        << "].";
    b_outStream << "\n";
    ((((b_outStream << "Error in ") << aInfo->fName) << " (line ")
     << aInfo->lineNo)
        << ")";
    throw std::runtime_error(b_outStream.str());
  } else {
    ((((((((outStream << "Index exceeds array dimensions. Index value ")
           << aIndexValue)
          << " exceeds valid range [")
         << aLoBound)
        << "-")
       << aHiBound)
      << "] for array \'")
     << aInfo->aName)
        << "\'.";
    outStream << "\n";
    ((((outStream << "Error in ") << aInfo->fName) << " (line ")
     << aInfo->lineNo)
        << ")";
    throw std::runtime_error(outStream.str());
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rtIntegerOverflowErrorN()
{
  std::stringstream outStream;
  outStream << "Integer overflow detected.\nEarly termination due to integer "
               "overflow.";
  outStream << "\n";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : const char *aString
// Return Type  : bool
//
bool rtIsNullOrEmptyString(const char *aString)
{
  return (aString == nullptr) ||
         ((static_cast<signed char>(*aString)) == '\x00');
}

} // namespace Codegen

//
// File trailer for Robot_Control_rtwutil.cpp
//
// [EOF]
//
