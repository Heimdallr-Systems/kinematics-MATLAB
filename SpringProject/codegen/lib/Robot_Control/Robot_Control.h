//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Robot_Control.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
extern void Robot_Control(const double r_II_B_d[3], const double Euler_d[3],
                          const double gamma_m[36], bool init_toggle,
                          const bool legs_on_gnd[4], double Theta1_d_out[4],
                          double Theta2_d_out[4], double Theta3_d_out[4],
                          double *phi_d_temp_out, double r_II_B_d_temp_out[3],
                          bool floor_toggle_out[4], bool legs_valid_out[4]);

void Robot_Control_init();

void endPhi_not_empty_init();

void endPoint_not_empty_init();

void floor_toggle_not_empty_init();

} // namespace Codegen

#endif
//
// File trailer for Robot_Control.h
//
// [EOF]
//
