//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Leg_Controller.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "Leg_Controller.h"
#include "Robot_Control_data.h"
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include "angle.h"
#include "atan2.h"
#include "cos.h"
#include "error.h"
#include "exp.h"
#include "mldivide.h"
#include "norm.h"
#include "power.h"
#include "rotz.h"
#include "sin.h"
#include "sqrt.h"
#include <stdio.h>

// Function Definitions
//
// This function operates similar to IK_Solver_Legs_Inertial, but instead is
//  intended to use the current T_I_B and r_II_B with a desired r_BB_c_d.
//  This should allow the leg desired to be moved act independent to the
//  position/tilt controller.
//  Works for one leg only!!!
//
// Arguments    : double r_II_c_d[3]
//                const double r_II_c_0[3]
//                const double T_I_B[9]
//                const double r_II_B[3]
//                unsigned char b_leg_index
//                double *Theta1
//                double *Theta2
//                double *Theta3
// Return Type  : void
//
namespace Codegen {
void Leg_Controller(double r_II_c_d[3], const double r_II_c_0[3],
                    const double T_I_B[9], const double r_II_B[3],
                    unsigned char b_leg_index, double *Theta1, double *Theta2,
                    double *Theta3)
{
  creal_T dc;
  double b_dv[9];
  double b_dv1[9];
  double b_dv2[9];
  double b_dv3[9];
  double b_dv6[9];
  double b_dv7[9];
  double b_dv8[9];
  double b_dv9[9];
  double b_dv10[3];
  double b_dv4[3];
  double b_dv5[3];
  double b_r_II_c_d[3];
  double dv11[3];
  double r_11_c_BL[3];
  double r_11_c_BL_2[3];
  double r_11_c_FL[3];
  double r_11_c_FL_2[3];
  double r_1prime1_2_BR[3];
  double r_1prime1_2_FR[3];
  double r_1prime1_c_BR[3];
  double r_1prime1_c_BR_2[3];
  double r_1prime1_c_FR[3];
  double r_1prime1_c_FR_2[3];
  double r_B1_c_BL[3];
  double r_B1_c_BR[3];
  double r_B1_c_FL[3];
  double r_B1_c_FR[3];
  double r_BB_c_BL[3];
  double r_BB_c_BR[3];
  double r_BB_c_FL[3];
  double r_BB_c_FR[3];
  double travel_dir_tmp[3];
  double Theta1_2;
  double Theta2_2;
  double Theta2_4;
  double Theta3_2;
  double Theta3_4;
  double d;
  double d10;
  double d12;
  double d13;
  double d14;
  double d16;
  double d17;
  double d18;
  double d19;
  double d20;
  double d21;
  double d23;
  double d24;
  double d25;
  double d26;
  double d28;
  double d29;
  double d3;
  double d30;
  double d31;
  double d32;
  double d33;
  double d34;
  double d35;
  double d36;
  double d37;
  double d38;
  double d39;
  double d4;
  double d40;
  double d41;
  double d42;
  double d43;
  double d44;
  double d45;
  double d46;
  double d47;
  double d48;
  double d5;
  double d7;
  double d9;
  double travel_dir_idx_0;
  double travel_dir_idx_1;
  double travel_dir_idx_2;
  int loop_toggle;
  unsigned short ii;
  bool T1_cond;
  bool hardstop_cond;
  bool hardstop_cond_2;
  //  Constants, known offsets
  //  known lengths of last two links
  //  Init these so that coder does not throw a hissy fit
  //  FIXME: This could actually cause problems. Check with Nick
  *Theta1 = 0.0;
  *Theta2 = 0.0;
  *Theta3 = 0.0;
  Theta2_2 = 0.0;
  Theta3_2 = 0.0;
  //
  travel_dir_tmp[0] = r_II_c_d[0] - r_II_c_0[0];
  travel_dir_tmp[1] = r_II_c_d[1] - r_II_c_0[1];
  travel_dir_tmp[2] = r_II_c_d[2] - r_II_c_0[2];
  d = coder::c_norm(travel_dir_tmp);
  travel_dir_idx_0 = travel_dir_tmp[0] / d;
  travel_dir_idx_1 = travel_dir_tmp[1] / d;
  travel_dir_idx_2 = travel_dir_tmp[2] / d;
  loop_toggle = 0;
  ii = 0U;
  while (loop_toggle == 0) {
    ii = _u16_u32_((static_cast<unsigned int>(ii)) + 1U);
    if ((static_cast<int>(ii)) == 1000) {
      (void)printf("Limit Reached");
      fflush(stdout);
      coder::error();
    }
    loop_toggle = 1;
    // initialize
    switch (b_leg_index) {
    case 1U: {
      double D_FR;
      double D_FR_2;
      double r_FR;
      double r_FR_2;
      double s_FR;
      double s_FR_2;
      //         %% FR LEG
      b_r_II_c_d[0] = r_II_c_d[0] - r_II_B[0];
      b_r_II_c_d[1] = r_II_c_d[1] - r_II_B[1];
      b_r_II_c_d[2] = r_II_c_d[2] - r_II_B[2];
      coder::mldivide(T_I_B, b_r_II_c_d, r_BB_c_FR);
      r_B1_c_FR[0] = r_BB_c_FR[0] - 0.12578;
      r_B1_c_FR[1] = r_BB_c_FR[1] - -0.12578;
      r_B1_c_FR[2] = r_BB_c_FR[2] - 0.0254;
      dc.re = 0.0;
      dc.im = coder::b_atan2(r_BB_c_FR[1] - -0.12578, r_BB_c_FR[0] - 0.12578) +
              1.5707963267948966;
      coder::b_exp(&dc);
      *Theta1 = coder::angle(dc);
      Theta1_2 = (*Theta1) + 3.1415926535897931;
      rotz(*Theta1, b_dv3);
      coder::mldivide(b_dv3, r_B1_c_FR, b_dv5);
      coder::mldivide(dv10, b_dv5, r_1prime1_c_FR);
      rotz((*Theta1) + 3.1415926535897931, b_dv9);
      coder::mldivide(b_dv9, r_B1_c_FR, dv11);
      coder::mldivide(dv10, dv11, r_1prime1_c_FR_2);
      coder::mldivide(dv10, dv1, r_1prime1_2_FR);
      r_FR = r_1prime1_c_FR[1] - r_1prime1_2_FR[1];
      r_FR_2 = r_1prime1_c_FR_2[1] - r_1prime1_2_FR[1];
      s_FR = r_1prime1_c_FR[2] - r_1prime1_2_FR[2];
      s_FR_2 = r_1prime1_c_FR_2[2] - r_1prime1_2_FR[2];
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
      D_FR =
          ((((r_FR * r_FR) + (s_FR * s_FR)) - 0.0225) - 0.041615999999999993) /
          0.061199999999999991;
      if (coder::fltpower_domain_error(r_FR_2)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(s_FR_2)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.15)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.204)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      D_FR_2 = ((((r_FR_2 * r_FR_2) + (s_FR_2 * s_FR_2)) - 0.0225) -
                0.041615999999999993) /
               0.061199999999999991;
      //  see if second solution possible
      if (D_FR_2 > 0.999) {
        Theta1_2 = 0.0;
        Theta3_4 = 0.0;
        Theta2_4 = 0.0;
      } else {
        double Theta3_FR_2_Temp_2;
        double d15;
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d15 = 1.0 - (D_FR_2 * D_FR_2);
        d16 = d15;
        coder::b_sqrt(&d16);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d26 = d15;
        coder::b_sqrt(&d26);
        Theta3_FR_2_Temp_2 = coder::b_atan2(-d26, D_FR_2);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d33 = d15;
        coder::b_sqrt(&d33);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d37 = d15;
        coder::b_sqrt(&d37);
        Theta3_4 = -coder::b_atan2(-d37, D_FR_2);
        d40 = Theta3_FR_2_Temp_2;
        coder::b_sin(&d40);
        d43 = Theta3_FR_2_Temp_2;
        coder::b_cos(&d43);
        Theta2_4 = -(coder::b_atan2(s_FR_2, r_FR_2) -
                     coder::b_atan2(0.204 * d40, (0.204 * d43) + 0.15));
      }
      if (D_FR > 0.999) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      } else {
        double Theta3_FR_2_Temp;
        double d27;
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d27 = 1.0 - (D_FR * D_FR);
        d28 = d27;
        coder::b_sqrt(&d28);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d34 = d27;
        coder::b_sqrt(&d34);
        Theta3_FR_2_Temp = coder::b_atan2(-d34, D_FR);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d38 = d27;
        coder::b_sqrt(&d38);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d44 = d27;
        coder::b_sqrt(&d44);
        Theta3_2 = -coder::b_atan2(-d44, D_FR);
        d46 = Theta3_FR_2_Temp;
        coder::b_sin(&d46);
        d48 = Theta3_FR_2_Temp;
        coder::b_cos(&d48);
        Theta2_2 = -(coder::b_atan2(s_FR, r_FR) -
                     coder::b_atan2(0.204 * d46, (0.204 * d48) + 0.15));
      }
      //             %% FL LEG
    } break;
    case 2U: {
      double D_FL;
      double D_FL_2;
      b_r_II_c_d[0] = r_II_c_d[0] - r_II_B[0];
      b_r_II_c_d[1] = r_II_c_d[1] - r_II_B[1];
      b_r_II_c_d[2] = r_II_c_d[2] - r_II_B[2];
      coder::mldivide(T_I_B, b_r_II_c_d, r_BB_c_FL);
      r_B1_c_FL[0] = r_BB_c_FL[0] - 0.12578;
      r_B1_c_FL[1] = r_BB_c_FL[1] - 0.12578;
      r_B1_c_FL[2] = r_BB_c_FL[2] - 0.0254;
      dc.re = 0.0;
      dc.im = coder::b_atan2(r_BB_c_FL[1] - 0.12578, r_BB_c_FL[0] - 0.12578) -
              1.5707963267948966;
      coder::b_exp(&dc);
      *Theta1 = coder::angle(dc);
      Theta1_2 = (*Theta1) + 3.1415926535897931;
      rotz(*Theta1, b_dv);
      coder::mldivide(b_dv, r_B1_c_FL, r_11_c_FL);
      rotz((*Theta1) + 3.1415926535897931, b_dv6);
      coder::mldivide(b_dv6, r_B1_c_FL, r_11_c_FL_2);
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
      if (coder::fltpower_domain_error(r_11_c_FL_2[1] - 0.054)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(r_11_c_FL_2[2])) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.15)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.204)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      D_FL_2 = (((((r_11_c_FL_2[1] - 0.054) * (r_11_c_FL_2[1] - 0.054)) +
                  (r_11_c_FL_2[2] * r_11_c_FL_2[2])) -
                 0.0225) -
                0.041615999999999993) /
               0.061199999999999991;
      if (D_FL_2 > 0.999) {
        Theta1_2 = 0.0;
        Theta3_4 = 0.0;
        Theta2_4 = 0.0;
      } else {
        double d1;
        if (coder::fltpower_domain_error(D_FL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d1 = 1.0 - (D_FL_2 * D_FL_2);
        d3 = d1;
        coder::b_sqrt(&d3);
        if (coder::fltpower_domain_error(D_FL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d5 = d1;
        coder::b_sqrt(&d5);
        Theta3_4 = coder::b_atan2(-d5, D_FL_2);
        d12 = Theta3_4;
        coder::b_sin(&d12);
        d17 = Theta3_4;
        coder::b_cos(&d17);
        Theta2_4 = coder::b_atan2(r_11_c_FL_2[2], r_11_c_FL_2[1] - 0.054) -
                   coder::b_atan2(0.204 * d12, (0.204 * d17) + 0.15);
      }
      if (D_FL > 0.999) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      } else {
        double d6;
        if (coder::fltpower_domain_error(D_FL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d6 = 1.0 - (D_FL * D_FL);
        d9 = d6;
        coder::b_sqrt(&d9);
        if (coder::fltpower_domain_error(D_FL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d18 = d6;
        coder::b_sqrt(&d18);
        Theta3_2 = coder::b_atan2(-d18, D_FL);
        d23 = Theta3_2;
        coder::b_sin(&d23);
        d29 = Theta3_2;
        coder::b_cos(&d29);
        Theta2_2 = coder::b_atan2(r_11_c_FL[2], r_11_c_FL[1] - 0.054) -
                   coder::b_atan2(0.204 * d23, (0.204 * d29) + 0.15);
      }
      //             %% BR LEG
    } break;
    case 3U: {
      double D_BR;
      double D_BR_2;
      double r_BR;
      double r_BR_2;
      double s_BR;
      double s_BR_2;
      b_r_II_c_d[0] = r_II_c_d[0] - r_II_B[0];
      b_r_II_c_d[1] = r_II_c_d[1] - r_II_B[1];
      b_r_II_c_d[2] = r_II_c_d[2] - r_II_B[2];
      coder::mldivide(T_I_B, b_r_II_c_d, r_BB_c_BR);
      r_B1_c_BR[0] = r_BB_c_BR[0] - -0.12578;
      r_B1_c_BR[1] = r_BB_c_BR[1] - -0.12578;
      r_B1_c_BR[2] = r_BB_c_BR[2] - 0.0254;
      dc.re = 0.0;
      dc.im = coder::b_atan2(r_BB_c_BR[1] - -0.12578, r_BB_c_BR[0] - -0.12578) +
              1.5707963267948966;
      coder::b_exp(&dc);
      *Theta1 = coder::angle(dc);
      Theta1_2 = (*Theta1) + 3.1415926535897931;
      rotz(*Theta1, b_dv1);
      coder::mldivide(b_dv1, r_B1_c_BR, b_dv4);
      coder::mldivide(dv10, b_dv4, r_1prime1_c_BR);
      rotz((*Theta1) + 3.1415926535897931, b_dv8);
      coder::mldivide(b_dv8, r_B1_c_BR, b_dv10);
      coder::mldivide(dv10, b_dv10, r_1prime1_c_BR_2);
      coder::mldivide(dv10, dv1, r_1prime1_2_BR);
      r_BR = r_1prime1_c_BR[1] - r_1prime1_2_BR[1];
      r_BR_2 = r_1prime1_c_BR_2[1] - r_1prime1_2_BR[1];
      s_BR = r_1prime1_c_BR[2] - r_1prime1_2_BR[2];
      s_BR_2 = r_1prime1_c_BR_2[2] - r_1prime1_2_BR[2];
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
      D_BR =
          ((((r_BR * r_BR) + (s_BR * s_BR)) - 0.0225) - 0.041615999999999993) /
          0.061199999999999991;
      if (coder::fltpower_domain_error(r_BR_2)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(s_BR_2)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.15)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.204)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      D_BR_2 = ((((r_BR_2 * r_BR_2) + (s_BR_2 * s_BR_2)) - 0.0225) -
                0.041615999999999993) /
               0.061199999999999991;
      if (D_BR_2 > 0.999) {
        Theta1_2 = 0.0;
        Theta3_4 = 0.0;
        Theta2_4 = 0.0;
      } else {
        double Theta3_BR_2_Temp_2;
        double d11;
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d11 = 1.0 - (D_BR_2 * D_BR_2);
        d13 = d11;
        coder::b_sqrt(&d13);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d21 = d11;
        coder::b_sqrt(&d21);
        Theta3_BR_2_Temp_2 = coder::b_atan2(-d21, D_BR_2);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d31 = d11;
        coder::b_sqrt(&d31);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d35 = d11;
        coder::b_sqrt(&d35);
        Theta3_4 = -coder::b_atan2(-d35, D_BR_2);
        d39 = Theta3_BR_2_Temp_2;
        coder::b_sin(&d39);
        d41 = Theta3_BR_2_Temp_2;
        coder::b_cos(&d41);
        Theta2_4 = -(coder::b_atan2(s_BR_2, r_BR_2) -
                     coder::b_atan2(0.204 * d39, (0.204 * d41) + 0.15));
      }
      if (D_BR > 0.999) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      } else {
        double Theta3_BR_2_Temp;
        double d22;
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d22 = 1.0 - (D_BR * D_BR);
        d24 = d22;
        coder::b_sqrt(&d24);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d32 = d22;
        coder::b_sqrt(&d32);
        Theta3_BR_2_Temp = coder::b_atan2(-d32, D_BR);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d36 = d22;
        coder::b_sqrt(&d36);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d42 = d22;
        coder::b_sqrt(&d42);
        Theta3_2 = -coder::b_atan2(-d42, D_BR);
        d45 = Theta3_BR_2_Temp;
        coder::b_sin(&d45);
        d47 = Theta3_BR_2_Temp;
        coder::b_cos(&d47);
        Theta2_2 = -(coder::b_atan2(s_BR, r_BR) -
                     coder::b_atan2(0.204 * d45, (0.204 * d47) + 0.15));
      }
      //             %% BL LEG
    } break;
    case 4U: {
      double D_BL;
      double D_BL_2;
      b_r_II_c_d[0] = r_II_c_d[0] - r_II_B[0];
      b_r_II_c_d[1] = r_II_c_d[1] - r_II_B[1];
      b_r_II_c_d[2] = r_II_c_d[2] - r_II_B[2];
      coder::mldivide(T_I_B, b_r_II_c_d, r_BB_c_BL);
      r_B1_c_BL[0] = r_BB_c_BL[0] - -0.12578;
      r_B1_c_BL[1] = r_BB_c_BL[1] - 0.12578;
      r_B1_c_BL[2] = r_BB_c_BL[2] - 0.0254;
      dc.re = 0.0;
      dc.im = coder::b_atan2(r_BB_c_BL[1] - 0.12578, r_BB_c_BL[0] - -0.12578) -
              1.5707963267948966;
      coder::b_exp(&dc);
      *Theta1 = coder::angle(dc);
      Theta1_2 = (*Theta1) + 3.1415926535897931;
      rotz(*Theta1, b_dv2);
      coder::mldivide(b_dv2, r_B1_c_BL, r_11_c_BL);
      rotz((*Theta1) + 3.1415926535897931, b_dv7);
      coder::mldivide(b_dv7, r_B1_c_BL, r_11_c_BL_2);
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
      if (coder::fltpower_domain_error(r_11_c_BL_2[1] - 0.054)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(r_11_c_BL_2[2])) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.15)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      if (coder::fltpower_domain_error(0.204)) {
        c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
      }
      D_BL_2 = (((((r_11_c_BL_2[1] - 0.054) * (r_11_c_BL_2[1] - 0.054)) +
                  (r_11_c_BL_2[2] * r_11_c_BL_2[2])) -
                 0.0225) -
                0.041615999999999993) /
               0.061199999999999991;
      if (D_BL_2 > 0.999) {
        Theta1_2 = 0.0;
        Theta3_4 = 0.0;
        Theta2_4 = 0.0;
      } else {
        double d2;
        if (coder::fltpower_domain_error(D_BL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d2 = 1.0 - (D_BL_2 * D_BL_2);
        d4 = d2;
        coder::b_sqrt(&d4);
        if (coder::fltpower_domain_error(D_BL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d7 = d2;
        coder::b_sqrt(&d7);
        Theta3_4 = coder::b_atan2(-d7, D_BL_2);
        d14 = Theta3_4;
        coder::b_sin(&d14);
        d19 = Theta3_4;
        coder::b_cos(&d19);
        Theta2_4 = coder::b_atan2(r_11_c_BL_2[2], r_11_c_BL_2[1] - 0.054) -
                   coder::b_atan2(0.204 * d14, (0.204 * d19) + 0.15);
      }
      if (D_BL > 0.999) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      } else {
        double d8;
        if (coder::fltpower_domain_error(D_BL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d8 = 1.0 - (D_BL * D_BL);
        d10 = d8;
        coder::b_sqrt(&d10);
        if (coder::fltpower_domain_error(D_BL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d20 = d8;
        coder::b_sqrt(&d20);
        Theta3_2 = coder::b_atan2(-d20, D_BL);
        d25 = Theta3_2;
        coder::b_sin(&d25);
        d30 = Theta3_2;
        coder::b_cos(&d30);
        Theta2_2 = coder::b_atan2(r_11_c_BL[2], r_11_c_BL[1] - 0.054) -
                   coder::b_atan2(0.204 * d25, (0.204 * d30) + 0.15);
      }
    } break;
    default:
      (void)printf("Leg_Index is set to an invalid value");
      fflush(stdout);
      coder::error();
      break;
    }
    //  if theta1 wraps around into robot
    switch (b_leg_index) {
    case 1U:
      if (((*Theta1) <= -1.5707963267948966) ||
          ((*Theta1) >= 3.1415926535897931)) {
        T1_cond = true;
      } else {
        T1_cond = false;
      }
      //  FR
      if (((*Theta1) > 1.3707963267948966) || ((*Theta1) < 0.2)) {
        hardstop_cond = true;
      } else {
        hardstop_cond = false;
      }
      if ((Theta1_2 > 1.3707963267948966) || (Theta1_2 < 0.2)) {
        hardstop_cond_2 = true;
      } else {
        hardstop_cond_2 = false;
      }
      break;
    case 2U:
      if (((*Theta1) <= -3.1415926535897931) ||
          ((*Theta1) >= 1.5707963267948966)) {
        T1_cond = true;
      } else {
        T1_cond = false;
      }
      // FL
      if (((*Theta1) > -0.2) || ((*Theta1) < -1.3707963267948966)) {
        hardstop_cond = true;
      } else {
        hardstop_cond = false;
      }
      if ((Theta1_2 > -0.2) || (Theta1_2 < -1.3707963267948966)) {
        hardstop_cond_2 = true;
      } else {
        hardstop_cond_2 = false;
      }
      break;
    case 3U:
      if (((*Theta1) <= -3.1415926535897931) ||
          ((*Theta1) >= 1.5707963267948966)) {
        T1_cond = true;
      } else {
        T1_cond = false;
      }
      //  BR
      if (((*Theta1) > -0.2) || ((*Theta1) < -1.3707963267948966)) {
        hardstop_cond = true;
      } else {
        hardstop_cond = false;
      }
      if ((Theta1_2 > -0.2) || (Theta1_2 < -1.3707963267948966)) {
        hardstop_cond_2 = true;
      } else {
        hardstop_cond_2 = false;
      }
      break;
    case 4U:
      if (((*Theta1) <= -1.5707963267948966) ||
          ((*Theta1) >= 3.1415926535897931)) {
        T1_cond = true;
      } else {
        T1_cond = false;
      }
      //  BL
      if (((*Theta1) > 1.3707963267948966) || ((*Theta1) < 0.2)) {
        hardstop_cond = true;
      } else {
        hardstop_cond = false;
      }
      if ((Theta1_2 > 1.3707963267948966) || (Theta1_2 < 0.2)) {
        hardstop_cond_2 = true;
      } else {
        hardstop_cond_2 = false;
      }
      break;
    default:
      (void)printf("Leg Index is not a allowed value");
      fflush(stdout);
      coder::error();
      break;
    }
    if (T1_cond) {
      *Theta1 = Theta1_2;
      *Theta2 = Theta2_4;
      *Theta3 = Theta3_4;
      if (hardstop_cond_2) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      }
    } else {
      *Theta2 = Theta2_2;
      *Theta3 = Theta3_2;
      if (hardstop_cond) {
        loop_toggle = 0;
        r_II_c_d[0] -= travel_dir_idx_0 * 0.01;
        r_II_c_d[1] -= travel_dir_idx_1 * 0.01;
        r_II_c_d[2] -= travel_dir_idx_2 * 0.01;
        r_II_c_d[2] = r_II_c_0[2];
      }
    }
  }
}

} // namespace Codegen

//
// File trailer for Leg_Controller.cpp
//
// [EOF]
//
