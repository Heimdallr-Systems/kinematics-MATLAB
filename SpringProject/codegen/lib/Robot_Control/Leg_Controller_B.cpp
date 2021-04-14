//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Leg_Controller_B.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "Leg_Controller_B.h"
#include "Robot_Control_data.h"
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include "angle.h"
#include "atan2.h"
#include "cos.h"
#include "error.h"
#include "exp.h"
#include "mldivide.h"
#include "power.h"
#include "rotz.h"
#include "sin.h"
#include "sqrt.h"
#include <stdio.h>

// Function Definitions
//
// This function is used to "reset" the legs to some r_BB_c position
//  Works for one leg only!!!
//
// Arguments    : const double r_BB_c[3]
//                unsigned char b_leg_index
//                double *Theta1
//                double *Theta2
//                double *Theta3
// Return Type  : void
//
namespace Codegen {
void Leg_Controller_B(const double r_BB_c[3], unsigned char b_leg_index,
                      double *Theta1, double *Theta2, double *Theta3)
{
  creal_T dc;
  double b_dv[9];
  double b_dv1[9];
  double b_dv2[9];
  double b_dv4[9];
  double b_dv3[3];
  double b_dv5[3];
  double r_11_c_BL[3];
  double r_11_c_FL[3];
  double r_1prime1_2_BR[3];
  double r_1prime1_2_FR[3];
  double r_1prime1_c_BR[3];
  double r_1prime1_c_FR[3];
  double r_B1_c_BL[3];
  double r_B1_c_BR[3];
  double r_B1_c_FL[3];
  double r_B1_c_FR[3];
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d17;
  double d18;
  double d19;
  double d2;
  double d20;
  double d21;
  double d22;
  double d23;
  double d3;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  //  Constants, known offsets
  //  known lengths of last two links
  switch (b_leg_index) {
  case 1U: {
    double D_FR;
    double Theta1_FR;
    double Theta3_FR_2_Temp;
    double d16;
    double r_FR;
    double s_FR;
    //  Vector from Base to Contact
    //  Vector from Base to Contact minus first joint offset from base.
    //  This is used to eliminate the offset so the frame can be centered on
    //  the first joint, allowing for arctangent function to be used to solve
    //  for theta1.
    r_B1_c_FR[0] = r_BB_c[0] - 0.12578;
    r_B1_c_FR[1] = r_BB_c[1] - -0.12578;
    r_B1_c_FR[2] = r_BB_c[2] - 0.0254;
    //  Solve for Theta1
    //  wrap theta1 between +-pi
    dc.re = 0.0;
    dc.im = coder::b_atan2(r_BB_c[1] - -0.12578, r_BB_c[0] - 0.12578) +
            1.5707963267948966;
    coder::b_exp(&dc);
    Theta1_FR = coder::angle(dc);
    //  Solve for Theta3
    //  Set up a positive frame centered on joint two, aligned with frame 1 in
    //  order to apply standard two-link manipulator solution (refer to
    //  textbook)
    rotz(Theta1_FR, b_dv4);
    coder::mldivide(b_dv4, r_B1_c_FR, b_dv5);
    coder::mldivide(dv10, b_dv5, r_1prime1_c_FR);
    //  vectors from joint 1 to joint 2
    //  transform vectors to positive, no offset frame to allow for standard two
    //  link problem
    coder::mldivide(dv10, dv1, r_1prime1_2_FR);
    //  solves for y-axis component
    r_FR = r_1prime1_c_FR[1] - r_1prime1_2_FR[1];
    //  solves for z-axis component
    s_FR = r_1prime1_c_FR[2] - r_1prime1_2_FR[2];
    //  cos(theta3)
    if (coder::fltpower_domain_error(r_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(s_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.15)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.204)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    D_FR = ((((r_FR * r_FR) + (s_FR * s_FR)) - 0.0225) - 0.041615999999999993) /
           0.061199999999999991;
    //  Theta 3 values measured with respect to the frame (frame 1 prime) used
    //  to solve standard 2-link problem frame . These will be used to solve for
    //  the theta 2 values
    if (coder::fltpower_domain_error(D_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d16 = 1.0 - (D_FR * D_FR);
    d17 = d16;
    coder::b_sqrt(&d17);
    if (coder::fltpower_domain_error(D_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d19 = d16;
    coder::b_sqrt(&d19);
    Theta3_FR_2_Temp = coder::b_atan2(-d19, D_FR);
    //  Theta 3 values measured in defined way (from zeroed position)
    if (coder::fltpower_domain_error(D_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d20 = d16;
    coder::b_sqrt(&d20);
    if (coder::fltpower_domain_error(D_FR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d21 = d16;
    coder::b_sqrt(&d21);
    *Theta3 = -coder::b_atan2(-d21, D_FR);
    //  Solve for Theta2
    d22 = Theta3_FR_2_Temp;
    coder::b_sin(&d22);
    d23 = Theta3_FR_2_Temp;
    coder::b_cos(&d23);
    *Theta2 = -(coder::b_atan2(s_FR, r_FR) -
                coder::b_atan2(0.204 * d22, (0.204 * d23) + 0.15));
    *Theta1 = Theta1_FR;
  } break;
  case 2U: {
    double D_FL;
    double Theta1_FL;
    double Theta3_FL_2;
    double d;
    r_B1_c_FL[0] = r_BB_c[0] - 0.12578;
    r_B1_c_FL[1] = r_BB_c[1] - 0.12578;
    r_B1_c_FL[2] = r_BB_c[2] - 0.0254;
    dc.re = 0.0;
    dc.im = coder::b_atan2(r_BB_c[1] - 0.12578, r_BB_c[0] - 0.12578) -
            1.5707963267948966;
    coder::b_exp(&dc);
    Theta1_FL = coder::angle(dc);
    rotz(Theta1_FL, b_dv);
    coder::mldivide(b_dv, r_B1_c_FL, r_11_c_FL);
    if (coder::fltpower_domain_error(r_11_c_FL[1] - 0.054)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(r_11_c_FL[2])) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.15)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.204)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    D_FL = (((((r_11_c_FL[1] - 0.054) * (r_11_c_FL[1] - 0.054)) +
              (r_11_c_FL[2] * r_11_c_FL[2])) -
             0.0225) -
            0.041615999999999993) /
           0.061199999999999991;
    if (coder::fltpower_domain_error(D_FL)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d = 1.0 - (D_FL * D_FL);
    d2 = d;
    coder::b_sqrt(&d2);
    if (coder::fltpower_domain_error(D_FL)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d5 = d;
    coder::b_sqrt(&d5);
    Theta3_FL_2 = coder::b_atan2(-d5, D_FL);
    d8 = Theta3_FL_2;
    coder::b_sin(&d8);
    d11 = Theta3_FL_2;
    coder::b_cos(&d11);
    *Theta2 = coder::b_atan2(r_11_c_FL[2], r_11_c_FL[1] - 0.054) -
              coder::b_atan2(0.204 * d8, (0.204 * d11) + 0.15);
    *Theta1 = Theta1_FL;
    *Theta3 = Theta3_FL_2;
  } break;
  case 3U: {
    double D_BR;
    double Theta1_BR;
    double Theta3_BR_2_Temp;
    double d4;
    double r_BR;
    double s_BR;
    r_B1_c_BR[0] = r_BB_c[0] - -0.12578;
    r_B1_c_BR[1] = r_BB_c[1] - -0.12578;
    r_B1_c_BR[2] = r_BB_c[2] - 0.0254;
    dc.re = 0.0;
    dc.im = coder::b_atan2(r_BB_c[1] - -0.12578, r_BB_c[0] - -0.12578) +
            1.5707963267948966;
    coder::b_exp(&dc);
    Theta1_BR = coder::angle(dc);
    rotz(Theta1_BR, b_dv1);
    coder::mldivide(b_dv1, r_B1_c_BR, b_dv3);
    coder::mldivide(dv10, b_dv3, r_1prime1_c_BR);
    coder::mldivide(dv10, dv1, r_1prime1_2_BR);
    r_BR = r_1prime1_c_BR[1] - r_1prime1_2_BR[1];
    s_BR = r_1prime1_c_BR[2] - r_1prime1_2_BR[2];
    if (coder::fltpower_domain_error(r_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(s_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.15)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.204)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    D_BR = ((((r_BR * r_BR) + (s_BR * s_BR)) - 0.0225) - 0.041615999999999993) /
           0.061199999999999991;
    if (coder::fltpower_domain_error(D_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d4 = 1.0 - (D_BR * D_BR);
    d6 = d4;
    coder::b_sqrt(&d6);
    if (coder::fltpower_domain_error(D_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d10 = d4;
    coder::b_sqrt(&d10);
    Theta3_BR_2_Temp = coder::b_atan2(-d10, D_BR);
    if (coder::fltpower_domain_error(D_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d13 = d4;
    coder::b_sqrt(&d13);
    if (coder::fltpower_domain_error(D_BR)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d14 = d4;
    coder::b_sqrt(&d14);
    *Theta3 = -coder::b_atan2(-d14, D_BR);
    d15 = Theta3_BR_2_Temp;
    coder::b_sin(&d15);
    d18 = Theta3_BR_2_Temp;
    coder::b_cos(&d18);
    *Theta2 = -(coder::b_atan2(s_BR, r_BR) -
                coder::b_atan2(0.204 * d15, (0.204 * d18) + 0.15));
    *Theta1 = Theta1_BR;
  } break;
  case 4U: {
    double D_BL;
    double Theta1_BL;
    double Theta3_BL_2;
    double d1;
    r_B1_c_BL[0] = r_BB_c[0] - -0.12578;
    r_B1_c_BL[1] = r_BB_c[1] - 0.12578;
    r_B1_c_BL[2] = r_BB_c[2] - 0.0254;
    dc.re = 0.0;
    dc.im = coder::b_atan2(r_BB_c[1] - 0.12578, r_BB_c[0] - -0.12578) -
            1.5707963267948966;
    coder::b_exp(&dc);
    Theta1_BL = coder::angle(dc);
    rotz(Theta1_BL, b_dv2);
    coder::mldivide(b_dv2, r_B1_c_BL, r_11_c_BL);
    if (coder::fltpower_domain_error(r_11_c_BL[1] - 0.054)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(r_11_c_BL[2])) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.15)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    if (coder::fltpower_domain_error(0.204)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    D_BL = (((((r_11_c_BL[1] - 0.054) * (r_11_c_BL[1] - 0.054)) +
              (r_11_c_BL[2] * r_11_c_BL[2])) -
             0.0225) -
            0.041615999999999993) /
           0.061199999999999991;
    if (coder::fltpower_domain_error(D_BL)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d1 = 1.0 - (D_BL * D_BL);
    d3 = d1;
    coder::b_sqrt(&d3);
    if (coder::fltpower_domain_error(D_BL)) {
      c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
    }
    d7 = d1;
    coder::b_sqrt(&d7);
    Theta3_BL_2 = coder::b_atan2(-d7, D_BL);
    d9 = Theta3_BL_2;
    coder::b_sin(&d9);
    d12 = Theta3_BL_2;
    coder::b_cos(&d12);
    *Theta2 = coder::b_atan2(r_11_c_BL[2], r_11_c_BL[1] - 0.054) -
              coder::b_atan2(0.204 * d9, (0.204 * d12) + 0.15);
    *Theta1 = Theta1_BL;
    *Theta3 = Theta3_BL_2;
  } break;
  default:
    (void)printf("leg_index is not set to a valid value");
    fflush(stdout);
    coder::error();
    break;
  }
}

} // namespace Codegen

//
// File trailer for Leg_Controller_B.cpp
//
// [EOF]
//
