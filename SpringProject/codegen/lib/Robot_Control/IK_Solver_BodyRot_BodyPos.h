//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: IK_Solver_BodyRot_BodyPos.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef IK_SOLVER_BODYROT_BODYPOS_H
#define IK_SOLVER_BODYROT_BODYPOS_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
void IK_Solver_BodyRot_BodyPos(const double r_BB_c[12], const double r_II_c[12],
                               const bool legs_on_gnd[4], double T_I_B[9],
                               double r_II_B[3]);

}

#endif
//
// File trailer for IK_Solver_BodyRot_BodyPos.h
//
// [EOF]
//
