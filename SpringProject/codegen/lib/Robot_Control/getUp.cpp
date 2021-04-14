//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getUp.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "getUp.h"
#include "error.h"
#include "norm.h"
#include <stdio.h>

// Function Definitions
//
// This function "resets" the robot if it has fallen or a kinematic solution
//  is unavailable. The legs all crunch up at once, then the robot lifts
//  straight up.
//  Start stage is 1
//  Intermediate stage is 2
//  End stage is 0
//  USE THIS FUNCTION IN A WHILE LOOP LIKE SO
//  while stage ~= 0
//      [Theta, stage] = getUp(Theta, stage);
//      FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
//  end
//  IF THE ERROR THRESHOLD FOR MOVING THE LEGS TAKES TOO LONG TO REACH,
//  MANUALLY CHANGE THE STAGE VALUE WITH A ITERATION COUNT CONDITION IN THE
//  WHILE LOOP.
//
// Arguments    : const double Theta[12]
//                unsigned char *stage
//                double Theta_d[12]
// Return Type  : void
//
namespace Codegen {
void getUp(const double Theta[12], unsigned char *stage, double Theta_d[12])
{
  static const double b_dv[12]{0.7854, -0.7854, -0.7854, 0.7854,
                               0.2542, -0.2542, 0.2542,  -0.2542,
                               1.5095, -1.5095, 1.5095,  -1.5095};
  static const double b_dv1[12]{
      0.78539816339744828,  -0.78539816339744828, -0.78539816339744828,
      0.78539816339744828,  -0.78539816339744828, 0.78539816339744828,
      -0.78539816339744828, 0.78539816339744828,  2.3561944901923448,
      -2.3561944901923448,  2.3561944901923448,   -2.3561944901923448};
  switch (*stage) {
  case 1U:
    //  constants for mid-step resting position
    //  crunch up
    //  FR
    Theta_d[0] = 0.78539816339744828;
    Theta_d[4] = -0.78539816339744828;
    Theta_d[8] = 2.3561944901923448;
    //  FL
    Theta_d[1] = -0.78539816339744828;
    Theta_d[5] = 0.78539816339744828;
    Theta_d[9] = -2.3561944901923448;
    //  BR
    Theta_d[2] = -0.78539816339744828;
    Theta_d[6] = -0.78539816339744828;
    Theta_d[10] = 2.3561944901923448;
    //  BL
    Theta_d[3] = 0.78539816339744828;
    Theta_d[7] = 0.78539816339744828;
    Theta_d[11] = -2.3561944901923448;
    *stage = 1U;
    //  calc error
    if ((coder::b_norm(Theta) - coder::b_norm(b_dv1)) < 0.3) {
      *stage = 2U;
    }
    break;
  case 2U:
    //  rise up
    Theta_d[0] = 0.7854;
    Theta_d[1] = -0.7854;
    Theta_d[2] = -0.7854;
    Theta_d[3] = 0.7854;
    Theta_d[4] = 0.2542;
    Theta_d[5] = -0.2542;
    Theta_d[6] = 0.2542;
    Theta_d[7] = -0.2542;
    Theta_d[8] = 1.5095;
    Theta_d[9] = -1.5095;
    Theta_d[10] = 1.5095;
    Theta_d[11] = -1.5095;
    *stage = 2U;
    if ((coder::b_norm(Theta) - coder::b_norm(b_dv)) < 0.3) {
      *stage = 0U;
    }
    break;
  default:
    (void)printf("Bad Index Input\n");
    fflush(stdout);
    coder::error();
    break;
  }
}

} // namespace Codegen

//
// File trailer for getUp.cpp
//
// [EOF]
//
