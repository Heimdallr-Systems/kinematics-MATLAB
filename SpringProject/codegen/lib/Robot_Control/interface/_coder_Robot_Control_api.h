//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_Robot_Control_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef _CODER_ROBOT_CONTROL_API_H
#define _CODER_ROBOT_CONTROL_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void CallTheDead(real_T Theta[12], real_T r_II_B_dead[3], real_T T_I_B_dead[9],
                 boolean_T firstCall, boolean_T legs_valid[4],
                 boolean_T prev_legs_valid[4], real_T r_II_c_dead[12]);

void CallTheDead_api(const mxArray *const prhs[7], int32_T nlhs,
                     const mxArray *plhs[4]);

void Robot_Control(real_T r_II_B_d[3], real_T Euler_d[3], real_T gamma_m[36],
                   boolean_T init_toggle, boolean_T legs_on_gnd[4],
                   real_T Theta1_d_out[4], real_T Theta2_d_out[4],
                   real_T Theta3_d_out[4], real_T *phi_d_temp_out,
                   real_T r_II_B_d_temp_out[3], boolean_T floor_toggle_out[4],
                   boolean_T legs_valid_out[4]);

void Robot_Control_api(const mxArray *const prhs[5], int32_T nlhs,
                       const mxArray *plhs[7]);

void Robot_Control_atexit();

void Robot_Control_initialize();

void Robot_Control_terminate();

void Robot_Control_xil_shutdown();

void Robot_Control_xil_terminate();

void getUp(real_T Theta[12], uint8_T *stage, real_T Theta_d[12]);

void getUp_api(const mxArray *const prhs[2], int32_T nlhs,
               const mxArray *plhs[2]);

#endif
//
// File trailer for _coder_Robot_Control_api.h
//
// [EOF]
//
