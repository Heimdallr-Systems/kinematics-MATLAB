//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Robot_Control_rtwutil.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef ROBOT_CONTROL_RTWUTIL_H
#define ROBOT_CONTROL_RTWUTIL_H

// Include Files
#include "Robot_Control_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
extern unsigned short _u16_u32_(unsigned int b);

extern void c_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void rtDynamicBoundsError(int aIndexValue, int aLoBound, int aHiBound,
                                 const rtBoundsCheckInfo *aInfo);

extern void rtIntegerOverflowErrorN();

extern bool rtIsNullOrEmptyString(const char *aString);

} // namespace Codegen

#endif
//
// File trailer for Robot_Control_rtwutil.h
//
// [EOF]
//
