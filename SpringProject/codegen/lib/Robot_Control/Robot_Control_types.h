//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Robot_Control_types.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef ROBOT_CONTROL_TYPES_H
#define ROBOT_CONTROL_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
namespace Codegen {
struct rtBoundsCheckInfo {
  int iFirst;
  int iLast;
  int lineNo;
  int colNo;
  const char *aName;
  const char *fName;
  const char *pName;
  int checkKind;
};

struct rtRunTimeErrorInfo {
  int lineNo;
  int colNo;
  const char *fName;
  const char *pName;
};

} // namespace Codegen

#endif
//
// File trailer for Robot_Control_types.h
//
// [EOF]
//
