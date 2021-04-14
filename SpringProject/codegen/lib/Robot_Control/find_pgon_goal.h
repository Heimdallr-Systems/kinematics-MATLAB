//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: find_pgon_goal.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef FIND_PGON_GOAL_H
#define FIND_PGON_GOAL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
void find_pgon_goal(const double r_II_c_FR[3], const double r_II_c_FL[3],
                    const double r_II_c_BR[3], const double r_II_c_BL[3],
                    double r_II_B[3], unsigned char lifted_leg, double *x,
                    double *y);

}

#endif
//
// File trailer for find_pgon_goal.h
//
// [EOF]
//
