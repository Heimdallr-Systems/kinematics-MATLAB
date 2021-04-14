//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: IK_Solver_BodyRot_BodyPos.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "IK_Solver_BodyRot_BodyPos.h"
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include "combineVectorElements.h"
#include "det.h"
#include "diag.h"
#include "findTrue4Elem.h"
#include "svd.h"

// Function Declarations
namespace Codegen {
static unsigned char _u8_s32_(int b);

}

// Function Definitions
//
// Arguments    : int b
// Return Type  : unsigned char
//
namespace Codegen {
static unsigned char _u8_s32_(int b)
{
  unsigned char a;
  a = static_cast<unsigned char>(b);
  if ((static_cast<int>(a)) != b) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

//
// This function is used to solve for the body rotation and the body
//  position assuming we've kept track of our contact point positions with
//  respect to the base sufficieintly. This solver only works if we know that
//  at least 3 legs are touching the ground. If only 2 are touching, then
//  there are infinite solutions.
//  r_BB_c = [r_BB_c_FR_x, r_BB_c_FL_x, r_BB_c_BR_x, r_BB_c_BL_x
//            r_BB_c_FR_y, r_BB_c_FL_y, r_BB_c_BR_y, r_BB_c_BL_y
//            r_BB_c_FR_z, r_BB_c_FL_z, r_BB_c_BR_z, r_BB_c_BL_z]
//  r_II_c = [r_II_c_FR_x, r_II_c_FL_x, r_II_c_BR_x, r_II_c_BL_x
//            r_II_c_FR_y, r_II_c_FL_y, r_II_c_BR_y, r_II_c_BL_y
//            r_II_c_FR_z, r_II_c_FL_z, r_II_c_BR_z, r_II_c_BL_z]
//  legs_on_gnd = [1 or 0,1 or 0,1 or 0,1 or 0] either 3 or 4 legs are touching
//  the ground leg notation is now in 1-4 since it may not always be the case
//  that a specific set of three legs may be touching the ground, instead the
//  three legs touching the ground may be a different combination. So 1-4 is
//  used to be the most general.
//
// Arguments    : const double r_BB_c[12]
//                const double r_II_c[12]
//                const bool legs_on_gnd[4]
//                double T_I_B[9]
//                double r_II_B[3]
// Return Type  : void
//
void IK_Solver_BodyRot_BodyPos(const double r_BB_c[12], const double r_II_c[12],
                               const bool legs_on_gnd[4], double T_I_B[9],
                               double r_II_B[3])
{
  static rtBoundsCheckInfo b_emlrtBCI{
      1,                           // iFirst
      4,                           // iLast
      28,                          // lineNo
      24,                          // colNo
      "r_BB_c",                    // aName
      "IK_Solver_BodyRot_BodyPos", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\IK_Solver_"
      "BodyRot_BodyPos.m", // pName
      0                    // checkKind
  };
  static rtBoundsCheckInfo c_emlrtBCI{
      1,                           // iFirst
      4,                           // iLast
      27,                          // lineNo
      24,                          // colNo
      "r_BB_c",                    // aName
      "IK_Solver_BodyRot_BodyPos", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\IK_Solver_"
      "BodyRot_BodyPos.m", // pName
      0                    // checkKind
  };
  static rtBoundsCheckInfo d_emlrtBCI{
      1,                           // iFirst
      4,                           // iLast
      26,                          // lineNo
      24,                          // colNo
      "r_BB_c",                    // aName
      "IK_Solver_BodyRot_BodyPos", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\IK_Solver_"
      "BodyRot_BodyPos.m", // pName
      0                    // checkKind
  };
  static rtBoundsCheckInfo emlrtBCI{
      1,                           // iFirst
      4,                           // iLast
      30,                          // lineNo
      28,                          // colNo
      "r_BB_c",                    // aName
      "IK_Solver_BodyRot_BodyPos", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\IK_Solver_"
      "BodyRot_BodyPos.m", // pName
      0                    // checkKind
  };
  double M[9];
  double S[9];
  double U[9];
  double V[9];
  double b_U[9];
  double b_r_II_c[9];
  double c_r_II_c[9];
  double d_r_II_c[9];
  double b_dv[3];
  double b_r_BB_c[3];
  double c_r_BB_c[3];
  double b_r_BB_c_tmp;
  double b_r_II_c_idx_0;
  double b_r_II_c_idx_1;
  double b_r_II_c_idx_1_tmp;
  double b_r_II_c_idx_2;
  double b_r_II_c_idx_2_tmp;
  double c_r_BB_c_tmp;
  double c_r_II_c_idx_0_tmp;
  double c_r_II_c_idx_1_tmp;
  double c_r_II_c_idx_2_tmp;
  double d_r_BB_c_tmp;
  double d_r_II_c_idx_0_tmp;
  double e_r_BB_c_tmp;
  double f_r_BB_c_tmp;
  double f_r_II_c_idx_0_tmp;
  double g_r_BB_c_tmp;
  double h_r_BB_c_tmp;
  double i_r_BB_c_tmp;
  double r_BB_c_tmp;
  double r_II_c_idx_0;
  double r_II_c_idx_1;
  double r_II_c_idx_1_tmp;
  double r_II_c_idx_2;
  double r_II_c_idx_2_tmp;
  int b_r_II_c_idx_0_tmp;
  int e_r_II_c_idx_0_tmp;
  int r_II_c_idx_0_tmp;
  unsigned char legs[4];
  bool b_legs_on_gnd[4];
  b_legs_on_gnd[0] = legs_on_gnd[0];
  b_legs_on_gnd[1] = legs_on_gnd[1];
  b_legs_on_gnd[2] = legs_on_gnd[2];
  b_legs_on_gnd[3] = legs_on_gnd[3];
  findTrue4Elem(legs_on_gnd, legs);
  if (((static_cast<int>(legs[0])) < 1) || ((static_cast<int>(legs[0])) > 4)) {
    rtDynamicBoundsError(static_cast<int>(legs[0]), 1, 4, &d_emlrtBCI);
  }
  if (((static_cast<int>(legs[1])) < 1) || ((static_cast<int>(legs[1])) > 4)) {
    rtDynamicBoundsError(static_cast<int>(legs[1]), 1, 4, &c_emlrtBCI);
  }
  if (((static_cast<int>(legs[2])) < 1) || ((static_cast<int>(legs[2])) > 4)) {
    rtDynamicBoundsError(static_cast<int>(legs[2]), 1, 4, &b_emlrtBCI);
  }
  if ((static_cast<int>(
          _u8_s32_(coder::combineVectorElements(b_legs_on_gnd)))) == 4) {
    if (((static_cast<int>(legs[3])) < 1) ||
        ((static_cast<int>(legs[3])) > 4)) {
      rtDynamicBoundsError(static_cast<int>(legs[3]), 1, 4, &emlrtBCI);
    }
  }
  //  apply Markley's Solution to Wahbah's Problem
  // product4 = r_II_i_4*(r_BB_i_4');
  // + product4
  r_II_c_idx_0_tmp = 3 * ((static_cast<int>(legs[0])) - 1);
  b_r_II_c_idx_0_tmp = 3 * ((static_cast<int>(legs[1])) - 1);
  c_r_II_c_idx_0_tmp = r_II_c[r_II_c_idx_0_tmp];
  d_r_II_c_idx_0_tmp = r_II_c[b_r_II_c_idx_0_tmp];
  r_II_c_idx_0 = c_r_II_c_idx_0_tmp - d_r_II_c_idx_0_tmp;
  r_BB_c_tmp = r_BB_c[r_II_c_idx_0_tmp];
  b_r_BB_c_tmp = r_BB_c[b_r_II_c_idx_0_tmp];
  b_r_BB_c[0] = r_BB_c_tmp - b_r_BB_c_tmp;
  e_r_II_c_idx_0_tmp = 3 * ((static_cast<int>(legs[2])) - 1);
  f_r_II_c_idx_0_tmp = r_II_c[e_r_II_c_idx_0_tmp];
  b_r_II_c_idx_0 = c_r_II_c_idx_0_tmp - f_r_II_c_idx_0_tmp;
  c_r_BB_c_tmp = r_BB_c[e_r_II_c_idx_0_tmp];
  c_r_BB_c[0] = r_BB_c_tmp - c_r_BB_c_tmp;
  r_II_c_idx_1_tmp = r_II_c[r_II_c_idx_0_tmp + 1];
  b_r_II_c_idx_1_tmp = r_II_c[b_r_II_c_idx_0_tmp + 1];
  r_II_c_idx_1 = r_II_c_idx_1_tmp - b_r_II_c_idx_1_tmp;
  d_r_BB_c_tmp = r_BB_c[r_II_c_idx_0_tmp + 1];
  e_r_BB_c_tmp = r_BB_c[b_r_II_c_idx_0_tmp + 1];
  b_r_BB_c[1] = d_r_BB_c_tmp - e_r_BB_c_tmp;
  c_r_II_c_idx_1_tmp = r_II_c[e_r_II_c_idx_0_tmp + 1];
  b_r_II_c_idx_1 = r_II_c_idx_1_tmp - c_r_II_c_idx_1_tmp;
  f_r_BB_c_tmp = r_BB_c[e_r_II_c_idx_0_tmp + 1];
  c_r_BB_c[1] = d_r_BB_c_tmp - f_r_BB_c_tmp;
  r_II_c_idx_2_tmp = r_II_c[r_II_c_idx_0_tmp + 2];
  b_r_II_c_idx_2_tmp = r_II_c[b_r_II_c_idx_0_tmp + 2];
  r_II_c_idx_2 = r_II_c_idx_2_tmp - b_r_II_c_idx_2_tmp;
  g_r_BB_c_tmp = r_BB_c[r_II_c_idx_0_tmp + 2];
  h_r_BB_c_tmp = r_BB_c[b_r_II_c_idx_0_tmp + 2];
  b_r_BB_c[2] = g_r_BB_c_tmp - h_r_BB_c_tmp;
  c_r_II_c_idx_2_tmp = r_II_c[e_r_II_c_idx_0_tmp + 2];
  b_r_II_c_idx_2 = r_II_c_idx_2_tmp - c_r_II_c_idx_2_tmp;
  i_r_BB_c_tmp = r_BB_c[e_r_II_c_idx_0_tmp + 2];
  c_r_BB_c[2] = g_r_BB_c_tmp - i_r_BB_c_tmp;
  for (int i{0}; i < 3; i++) {
    double d;
    double d1;
    int b_r_II_c_tmp;
    int r_II_c_tmp;
    d = b_r_BB_c[i];
    b_r_II_c[3 * i] = r_II_c_idx_0 * d;
    d1 = c_r_BB_c[i];
    c_r_II_c[3 * i] = b_r_II_c_idx_0 * d1;
    r_II_c_tmp = (3 * i) + 1;
    b_r_II_c[r_II_c_tmp] = r_II_c_idx_1 * d;
    c_r_II_c[r_II_c_tmp] = b_r_II_c_idx_1 * d1;
    b_r_II_c_tmp = (3 * i) + 2;
    b_r_II_c[b_r_II_c_tmp] = r_II_c_idx_2 * d;
    c_r_II_c[b_r_II_c_tmp] = b_r_II_c_idx_2 * d1;
  }
  r_II_c_idx_0 = d_r_II_c_idx_0_tmp - f_r_II_c_idx_0_tmp;
  b_r_BB_c[0] = b_r_BB_c_tmp - c_r_BB_c_tmp;
  r_II_c_idx_1 = b_r_II_c_idx_1_tmp - c_r_II_c_idx_1_tmp;
  b_r_BB_c[1] = e_r_BB_c_tmp - f_r_BB_c_tmp;
  r_II_c_idx_2 = b_r_II_c_idx_2_tmp - c_r_II_c_idx_2_tmp;
  b_r_BB_c[2] = h_r_BB_c_tmp - i_r_BB_c_tmp;
  for (int i1{0}; i1 < 3; i1++) {
    double d2;
    int c_r_II_c_tmp;
    int d_r_II_c_tmp;
    d2 = b_r_BB_c[i1];
    d_r_II_c[3 * i1] =
        (b_r_II_c[3 * i1] + c_r_II_c[3 * i1]) + (r_II_c_idx_0 * d2);
    c_r_II_c_tmp = (3 * i1) + 1;
    d_r_II_c[c_r_II_c_tmp] =
        (b_r_II_c[c_r_II_c_tmp] + c_r_II_c[c_r_II_c_tmp]) + (r_II_c_idx_1 * d2);
    d_r_II_c_tmp = (3 * i1) + 2;
    d_r_II_c[d_r_II_c_tmp] =
        (b_r_II_c[d_r_II_c_tmp] + c_r_II_c[d_r_II_c_tmp]) + (r_II_c_idx_2 * d2);
  }
  double d3;
  double d4;
  coder::svd(d_r_II_c, U, S, V);
  d3 = coder::det(U);
  d4 = coder::det(V);
  b_dv[0] = 1.0;
  b_dv[1] = 1.0;
  b_dv[2] = d3 * d4;
  coder::diag(b_dv, M);
  for (int i2{0}; i2 < 3; i2++) {
    double d10;
    double d11;
    double d5;
    double d6;
    double d7;
    double d8;
    double d9;
    d5 = U[i2];
    d6 = U[i2 + 3];
    d7 = U[i2 + 6];
    for (int i3{0}; i3 < 3; i3++) {
      b_U[i2 + (3 * i3)] =
          ((d5 * M[3 * i3]) + (d6 * M[(3 * i3) + 1])) + (d7 * M[(3 * i3) + 2]);
    }
    d8 = 0.0;
    d9 = b_U[i2];
    d10 = b_U[i2 + 3];
    d11 = b_U[i2 + 6];
    for (int i4{0}; i4 < 3; i4++) {
      double d12;
      d12 = ((d9 * V[i4]) + (d10 * V[i4 + 3])) + (d11 * V[i4 + 6]);
      T_I_B[i2 + (3 * i4)] = d12;
      d8 += d12 * r_BB_c[i4 + r_II_c_idx_0_tmp];
    }
    r_II_B[i2] = r_II_c[i2 + r_II_c_idx_0_tmp] - d8;
  }
}

} // namespace Codegen

//
// File trailer for IK_Solver_BodyRot_BodyPos.cpp
//
// [EOF]
//
