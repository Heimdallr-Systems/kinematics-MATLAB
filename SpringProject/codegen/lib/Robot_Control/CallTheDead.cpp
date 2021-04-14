//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CallTheDead.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "CallTheDead.h"
#include "CPos_wrt_B.h"
#include "CPos_wrt_I.h"
#include "IK_Solver_BodyRot_BodyPos.h"
#include "isequal.h"

// Function Definitions
//
// Arguments    : const double Theta[12]
//                double r_II_B_dead[3]
//                double T_I_B_dead[9]
//                bool firstCall
//                const bool b_legs_valid[4]
//                bool prev_legs_valid[4]
//                double r_II_c_dead[12]
// Return Type  : void
//
namespace Codegen {
void CallTheDead(const double Theta[12], double r_II_B_dead[3],
                 double T_I_B_dead[9], bool firstCall,
                 const bool b_legs_valid[4], bool prev_legs_valid[4],
                 double r_II_c_dead[12])
{
  static const double b_dv1[4]{0.0, 1.0, 1.0, 1.0};
  static const double b_dv2[4]{1.0, 0.0, 1.0, 1.0};
  static const double b_dv3[4]{1.0, 1.0, 0.0, 1.0};
  static const double b_dv4[4]{1.0, 1.0, 1.0, 0.0};
  double r_BB_c_dead[12];
  double b_dv[4];
  double r_BB_c_BL_dead[3];
  double r_BB_c_BR_dead[3];
  double r_BB_c_FL_dead[3];
  double r_BB_c_FR_dead[3];
  double r_II_c_BL_dead[3];
  double r_II_c_BR_dead[3];
  double r_II_c_FL_dead[3];
  double r_II_c_FR_dead[3];
  bool guard1{false};
  // %% DEAD RECKONING %%%
  // Theta = [Theta1;Theta2;Theta3];
  // r_II_c_dead = [r_II_c_FR_dead, r_II_c_FL_dead, r_II_c_BR_dead,
  // r_II_c_BL_dead]
  CPos_wrt_B(*((double(*)[4])(&Theta[0])), *((double(*)[4])(&Theta[4])),
             *((double(*)[4])(&Theta[8])), r_BB_c_FR_dead, r_BB_c_FL_dead,
             r_BB_c_BR_dead, r_BB_c_BL_dead);
  r_BB_c_dead[0] = r_BB_c_FR_dead[0];
  r_BB_c_dead[3] = r_BB_c_FL_dead[0];
  r_BB_c_dead[6] = r_BB_c_BR_dead[0];
  r_BB_c_dead[9] = r_BB_c_BL_dead[0];
  r_II_c_FR_dead[0] = r_II_c_dead[0];
  r_II_c_FL_dead[0] = r_II_c_dead[3];
  r_II_c_BR_dead[0] = r_II_c_dead[6];
  r_II_c_BL_dead[0] = r_II_c_dead[9];
  r_BB_c_dead[1] = r_BB_c_FR_dead[1];
  r_BB_c_dead[4] = r_BB_c_FL_dead[1];
  r_BB_c_dead[7] = r_BB_c_BR_dead[1];
  r_BB_c_dead[10] = r_BB_c_BL_dead[1];
  r_II_c_FR_dead[1] = r_II_c_dead[1];
  r_II_c_FL_dead[1] = r_II_c_dead[4];
  r_II_c_BR_dead[1] = r_II_c_dead[7];
  r_II_c_BL_dead[1] = r_II_c_dead[10];
  r_BB_c_dead[2] = r_BB_c_FR_dead[2];
  r_BB_c_dead[5] = r_BB_c_FL_dead[2];
  r_BB_c_dead[8] = r_BB_c_BR_dead[2];
  r_BB_c_dead[11] = r_BB_c_BL_dead[2];
  r_II_c_FR_dead[2] = r_II_c_dead[2];
  r_II_c_FL_dead[2] = r_II_c_dead[5];
  r_II_c_BR_dead[2] = r_II_c_dead[8];
  r_II_c_BL_dead[2] = r_II_c_dead[11];
  if (firstCall) {
    //  must start robot in no-tilt orientation
    CPos_wrt_I(*((double(*)[4])(&Theta[0])), *((double(*)[4])(&Theta[4])),
               *((double(*)[4])(&Theta[8])), T_I_B_dead, r_II_B_dead,
               r_II_c_FR_dead, r_II_c_FL_dead, r_II_c_BR_dead, r_II_c_BL_dead);
    r_II_c_dead[0] = r_II_c_FR_dead[0];
    r_II_c_dead[3] = r_II_c_FL_dead[0];
    r_II_c_dead[6] = r_II_c_BR_dead[0];
    r_II_c_dead[9] = r_II_c_BL_dead[0];
    r_II_c_dead[1] = r_II_c_FR_dead[1];
    r_II_c_dead[4] = r_II_c_FL_dead[1];
    r_II_c_dead[7] = r_II_c_BR_dead[1];
    r_II_c_dead[10] = r_II_c_BL_dead[1];
    r_II_c_dead[2] = r_II_c_FR_dead[2];
    r_II_c_dead[5] = r_II_c_FL_dead[2];
    r_II_c_dead[8] = r_II_c_BR_dead[2];
    r_II_c_dead[11] = r_II_c_BL_dead[2];
  }
  b_dv[0] = 1.0;
  b_dv[1] = 1.0;
  b_dv[2] = 1.0;
  b_dv[3] = 1.0;
  if (coder::isequal(b_dv, b_legs_valid)) {
    b_dv[0] = 1.0;
    b_dv[1] = 1.0;
    b_dv[2] = 1.0;
    b_dv[3] = 1.0;
    if (coder::isequal(b_dv, prev_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, b_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    }
  }
  guard1 = false;
  if (coder::isequal(b_dv1, b_legs_valid)) {
    guard1 = true;
  } else if (coder::isequal(b_dv1, prev_legs_valid)) {
    guard1 = true;
  } else {
    /* no actions */
  }
  if (guard1) {
    double d;
    double d1;
    double d2;
    if (coder::isequal(b_dv1, b_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, b_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else if (coder::isequal(b_dv1, prev_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else {
      /* no actions */
    }
    d = r_BB_c_FR_dead[0];
    d1 = r_BB_c_FR_dead[1];
    d2 = r_BB_c_FR_dead[2];
    for (int i{0}; i < 3; i++) {
      r_II_c_FR_dead[i] =
          r_II_B_dead[i] + (((T_I_B_dead[i] * d) + (T_I_B_dead[i + 3] * d1)) +
                            (T_I_B_dead[i + 6] * d2));
    }
  }
  guard1 = false;
  if (coder::isequal(b_dv2, b_legs_valid)) {
    guard1 = true;
  } else if (coder::isequal(b_dv2, prev_legs_valid)) {
    guard1 = true;
  } else {
    /* no actions */
  }
  if (guard1) {
    // falling edge (or rising) detection to finalize calculations
    if (coder::isequal(b_dv2, b_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, b_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else if (coder::isequal(b_dv2, prev_legs_valid)) {
      double d3;
      double d4;
      double d5;
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid,
                                T_I_B_dead, r_II_B_dead);
      d3 = r_BB_c_FL_dead[0];
      d4 = r_BB_c_FL_dead[1];
      d5 = r_BB_c_FL_dead[2];
      for (int i1{0}; i1 < 3; i1++) {
        r_II_c_FL_dead[i1] =
            r_II_B_dead[i1] +
            (((T_I_B_dead[i1] * d3) + (T_I_B_dead[i1 + 3] * d4)) +
             (T_I_B_dead[i1 + 6] * d5));
      }
    } else {
      /* no actions */
    }
  }
  guard1 = false;
  if (coder::isequal(b_dv3, b_legs_valid)) {
    guard1 = true;
  } else if (coder::isequal(b_dv3, prev_legs_valid)) {
    guard1 = true;
  } else {
    /* no actions */
  }
  if (guard1) {
    double d6;
    double d7;
    double d8;
    if (coder::isequal(b_dv3, b_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, b_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else if (coder::isequal(b_dv3, prev_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else {
      /* no actions */
    }
    d6 = r_BB_c_BR_dead[0];
    d7 = r_BB_c_BR_dead[1];
    d8 = r_BB_c_BR_dead[2];
    for (int i2{0}; i2 < 3; i2++) {
      r_II_c_BR_dead[i2] =
          r_II_B_dead[i2] +
          (((T_I_B_dead[i2] * d6) + (T_I_B_dead[i2 + 3] * d7)) +
           (T_I_B_dead[i2 + 6] * d8));
    }
  }
  guard1 = false;
  if (coder::isequal(b_dv4, b_legs_valid)) {
    guard1 = true;
  } else if (coder::isequal(b_dv4, prev_legs_valid)) {
    guard1 = true;
  } else {
    /* no actions */
  }
  if (guard1) {
    double d10;
    double d11;
    double d9;
    if (coder::isequal(b_dv4, b_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, b_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else if (coder::isequal(b_dv4, prev_legs_valid)) {
      IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid,
                                T_I_B_dead, r_II_B_dead);
    } else {
      /* no actions */
    }
    d9 = r_BB_c_BL_dead[0];
    d10 = r_BB_c_BL_dead[1];
    d11 = r_BB_c_BL_dead[2];
    for (int i3{0}; i3 < 3; i3++) {
      r_II_c_BL_dead[i3] =
          r_II_B_dead[i3] +
          (((T_I_B_dead[i3] * d9) + (T_I_B_dead[i3 + 3] * d10)) +
           (T_I_B_dead[i3 + 6] * d11));
    }
  }
  r_II_c_dead[0] = r_II_c_FR_dead[0];
  r_II_c_dead[3] = r_II_c_FL_dead[0];
  r_II_c_dead[6] = r_II_c_BR_dead[0];
  r_II_c_dead[9] = r_II_c_BL_dead[0];
  r_II_c_dead[1] = r_II_c_FR_dead[1];
  r_II_c_dead[4] = r_II_c_FL_dead[1];
  r_II_c_dead[7] = r_II_c_BR_dead[1];
  r_II_c_dead[10] = r_II_c_BL_dead[1];
  r_II_c_dead[2] = r_II_c_FR_dead[2];
  r_II_c_dead[5] = r_II_c_FL_dead[2];
  r_II_c_dead[8] = r_II_c_BR_dead[2];
  r_II_c_dead[11] = r_II_c_BL_dead[2];
  prev_legs_valid[0] = b_legs_valid[0];
  prev_legs_valid[1] = b_legs_valid[1];
  prev_legs_valid[2] = b_legs_valid[2];
  prev_legs_valid[3] = b_legs_valid[3];
}

} // namespace Codegen

//
// File trailer for CallTheDead.cpp
//
// [EOF]
//
