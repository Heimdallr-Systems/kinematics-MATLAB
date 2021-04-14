//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_Robot_Control_mex.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef _CODER_ROBOT_CONTROL_MEX_H
#define _CODER_ROBOT_CONTROL_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
void CallTheDead_mexFunction(int32_T nlhs, mxArray *plhs[4], int32_T nrhs,
                             const mxArray *prhs[7]);

void Robot_Control_mexFunction(int32_T nlhs, mxArray *plhs[7], int32_T nrhs,
                               const mxArray *prhs[5]);

void getUp_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                       const mxArray *prhs[2]);

MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

#endif
//
// File trailer for _coder_Robot_Control_mex.h
//
// [EOF]
//
