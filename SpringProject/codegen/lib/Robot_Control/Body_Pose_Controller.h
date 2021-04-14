//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Body_Pose_Controller.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef BODY_POSE_CONTROLLER_H
#define BODY_POSE_CONTROLLER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
void Body_Pose_Controller(const double r_II_c[12], const double T_I_B[9],
                          double r_II_B_d[3], const double r_II_B_0[3],
                          const bool legs_on_gnd[4], double Theta1[4],
                          double Theta2[4], double Theta3[4]);

}

#endif
//
// File trailer for Body_Pose_Controller.h
//
// [EOF]
//
