//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Body_Pose_Controller.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "Body_Pose_Controller.h"
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
// This function is used to solve for IK solutions of the robot's legs when
//  measurements in terms of the inertial frame are known. Use this function
//  when solving for tilt under the flat plane assumption.
//  r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];
//  r_II_c_?? = [x_II_c; y_II_c; z_II_c]
//  BodyRot = zyx rotation of body
//  r_II_B_d = vector from inertial to body
//  Returns: Theta1 of each leg, both solutions for theta2 of each leg, both
//  solutions for theta3 of each leg
//
// Arguments    : const double r_II_c[12]
//                const double T_I_B[9]
//                double r_II_B_d[3]
//                const double r_II_B_0[3]
//                const bool legs_on_gnd[4]
//                double Theta1[4]
//                double Theta2[4]
//                double Theta3[4]
// Return Type  : void
//
namespace Codegen {
void Body_Pose_Controller(const double r_II_c[12], const double T_I_B[9],
                          double r_II_B_d[3], const double r_II_B_0[3],
                          const bool legs_on_gnd[4], double Theta1[4],
                          double Theta2[4], double Theta3[4])
{
  creal_T dc;
  double b_dv[9];
  double b_dv10[9];
  double b_dv2[9];
  double b_dv4[9];
  double b_dv5[9];
  double b_dv6[9];
  double b_dv8[9];
  double dv11[9];
  double b_dv1[3];
  double b_dv3[3];
  double b_dv7[3];
  double b_dv9[3];
  double b_r_II_c[3];
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
  double Theta1_2_idx_0;
  double Theta1_2_idx_1;
  double Theta1_2_idx_2;
  double Theta1_2_idx_3;
  double Theta2_2_idx_0;
  double Theta2_2_idx_1;
  double Theta2_2_idx_2;
  double Theta2_2_idx_3;
  double Theta2_4_idx_0;
  double Theta2_4_idx_1;
  double Theta2_4_idx_2;
  double Theta2_4_idx_3;
  double Theta3_2_idx_0;
  double Theta3_2_idx_1;
  double Theta3_2_idx_2;
  double Theta3_2_idx_3;
  double Theta3_4_idx_0;
  double Theta3_4_idx_1;
  double Theta3_4_idx_2;
  double Theta3_4_idx_3;
  double d;
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d16;
  double d17;
  double d18;
  double d19;
  double d2;
  double d21;
  double d22;
  double d23;
  double d25;
  double d26;
  double d27;
  double d28;
  double d29;
  double d30;
  double d32;
  double d33;
  double d34;
  double d35;
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
  double d49;
  double d5;
  double d50;
  double d51;
  double d52;
  double d53;
  double d54;
  double d55;
  double d56;
  double d6;
  double d9;
  double travel_dir_idx_0;
  double travel_dir_idx_1;
  double travel_dir_idx_2;
  unsigned short ii;
  bool T1_cond_idx_0;
  bool T1_cond_idx_1;
  bool T1_cond_idx_2;
  bool T1_cond_idx_3;
  bool loop_toggle;
  //  Constants, known offsets
  //  known lengths of last two links
  loop_toggle = false;
  // initialize
  Theta1[0] = 0.0;
  Theta1_2_idx_0 = 0.0;
  Theta2_2_idx_0 = 0.0;
  Theta2_4_idx_0 = 0.0;
  Theta3_2_idx_0 = 0.0;
  Theta3_4_idx_0 = 0.0;
  Theta1[1] = 0.0;
  Theta1_2_idx_1 = 0.0;
  Theta2_2_idx_1 = 0.0;
  Theta2_4_idx_1 = 0.0;
  Theta3_2_idx_1 = 0.0;
  Theta3_4_idx_1 = 0.0;
  Theta1[2] = 0.0;
  Theta1_2_idx_2 = 0.0;
  Theta2_2_idx_2 = 0.0;
  Theta2_4_idx_2 = 0.0;
  Theta3_2_idx_2 = 0.0;
  Theta3_4_idx_2 = 0.0;
  Theta1[3] = 0.0;
  Theta1_2_idx_3 = 0.0;
  Theta2_2_idx_3 = 0.0;
  Theta2_4_idx_3 = 0.0;
  Theta3_2_idx_3 = 0.0;
  Theta3_4_idx_3 = 0.0;
  //  Reference for what the elements are for each variable
  //  Theta1(:,1) = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL]
  //  Theta1_2(:,1) = [Theta1_FR_2, Theta1_FL_2, Theta1_BR_2, Theta1_BL_2]
  //  Theta2(:,1) = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]
  //  Theta2_2(:,1) =  [Theta2_FR_2, Theta2_FL_2, Theta2_BR_2, Theta2_BL_2]
  //  Theta2_3(:,1) = [Theta2_FR_3, Theta2_FL_3, Theta2_BR_3, Theta2_BL_3]
  //  Theta2_4(:,1) = [Theta2_FR_4, Theta2_FL_4, Theta2_BR_4, Theta2_BL_4]
  //  Theta3(:,1) = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]
  //  Theta3_2(:,1) =  [Theta3_FR_2, Theta3_FL_2, Theta3_BR_2, Theta3_BL_2]
  //  Theta3_3(:,1) = [Theta3_FR_3, Theta3_FL_3, Theta3_BR_3, Theta3_BL_3]
  //  Theta3_4(:,1) = [Theta3_FR_4, Theta3_FL_4, Theta3_BR_4, Theta3_BL_4]
  travel_dir_tmp[0] = r_II_B_d[0] - r_II_B_0[0];
  travel_dir_tmp[1] = r_II_B_d[1] - r_II_B_0[1];
  travel_dir_tmp[2] = r_II_B_d[2] - r_II_B_0[2];
  d = coder::c_norm(travel_dir_tmp);
  travel_dir_idx_0 = travel_dir_tmp[0] / d;
  travel_dir_idx_1 = travel_dir_tmp[1] / d;
  travel_dir_idx_2 = travel_dir_tmp[2] / d;
  ii = 0U;
  while (!loop_toggle) {
    ii = _u16_u32_((static_cast<unsigned int>(ii)) + 1U);
    if ((static_cast<int>(ii)) == 1000) {
      (void)printf("Limit Reached\n");
      fflush(stdout);
      coder::error();
    }
    //     %% FR LEG
    loop_toggle = true;
    if (legs_on_gnd[0]) {
      double D_FR;
      double D_FR_2;
      double r_FR;
      double r_FR_2;
      double s_FR;
      double s_FR_2;
      b_r_II_c[0] = r_II_c[0] - r_II_B_d[0];
      b_r_II_c[1] = r_II_c[1] - r_II_B_d[1];
      b_r_II_c[2] = r_II_c[2] - r_II_B_d[2];
      coder::mldivide(T_I_B, b_r_II_c, r_BB_c_FR);
      r_B1_c_FR[0] = r_BB_c_FR[0] - 0.12578;
      r_B1_c_FR[1] = r_BB_c_FR[1] - -0.12578;
      r_B1_c_FR[2] = r_BB_c_FR[2] - 0.0254;
      dc.re = 0.0;
      dc.im = coder::b_atan2(r_BB_c_FR[1] - -0.12578, r_BB_c_FR[0] - 0.12578) +
              1.5707963267948966;
      coder::b_exp(&dc);
      Theta1[0] = coder::angle(dc);
      Theta1_2_idx_0 = Theta1[0] + 3.1415926535897931;
      rotz(Theta1[0], b_dv);
      coder::mldivide(b_dv, r_B1_c_FR, b_dv1);
      coder::mldivide(dv10, b_dv1, r_1prime1_c_FR);
      rotz(Theta1[0] + 3.1415926535897931, b_dv2);
      coder::mldivide(b_dv2, r_B1_c_FR, b_dv3);
      coder::mldivide(dv10, b_dv3, r_1prime1_c_FR_2);
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
      if (D_FR > 0.999) {
        loop_toggle = false;
        r_II_B_d[0] -= travel_dir_idx_0 * 0.001;
        r_II_B_d[1] -= travel_dir_idx_1 * 0.001;
        r_II_B_d[2] -= travel_dir_idx_2 * 0.001;
        //              r_II_B_d(3) = r_II_B_0(3);
      } else {
        double Theta3_FR_2_Temp;
        double Theta3_FR_Temp;
        double d1;
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d1 = 1.0 - (D_FR * D_FR);
        d2 = d1;
        coder::b_sqrt(&d2);
        Theta3_FR_Temp = coder::b_atan2(d2, D_FR);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d5 = d1;
        coder::b_sqrt(&d5);
        Theta3_FR_2_Temp = coder::b_atan2(-d5, D_FR);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d12 = d1;
        coder::b_sqrt(&d12);
        if (coder::fltpower_domain_error(D_FR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d16 = d1;
        coder::b_sqrt(&d16);
        Theta3_2_idx_0 = -coder::b_atan2(-d16, D_FR);
        d18 = Theta3_FR_Temp;
        coder::b_sin(&d18);
        d23 = Theta3_FR_Temp;
        coder::b_cos(&d23);
        d27 = Theta3_FR_2_Temp;
        coder::b_sin(&d27);
        d32 = Theta3_FR_2_Temp;
        coder::b_cos(&d32);
        Theta2_2_idx_0 = -(coder::b_atan2(s_FR, r_FR) -
                           coder::b_atan2(0.204 * d27, (0.204 * d32) + 0.15));
      }
      //  see if second solution possible
      if (D_FR_2 > 0.999) {
        Theta1_2_idx_0 = 0.0;
        Theta3_4_idx_0 = 0.0;
        Theta2_4_idx_0 = 0.0;
      } else {
        double Theta3_FR_2_Temp_2;
        double d8;
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d8 = 1.0 - (D_FR_2 * D_FR_2);
        d10 = d8;
        coder::b_sqrt(&d10);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d14 = d8;
        coder::b_sqrt(&d14);
        Theta3_FR_2_Temp_2 = coder::b_atan2(-d14, D_FR_2);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d19 = d8;
        coder::b_sqrt(&d19);
        if (coder::fltpower_domain_error(D_FR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d28 = d8;
        coder::b_sqrt(&d28);
        Theta3_4_idx_0 = -coder::b_atan2(-d28, D_FR_2);
        d34 = Theta3_FR_2_Temp_2;
        coder::b_sin(&d34);
        d38 = Theta3_FR_2_Temp_2;
        coder::b_cos(&d38);
        Theta2_4_idx_0 = -(coder::b_atan2(s_FR_2, r_FR_2) -
                           coder::b_atan2(0.204 * d34, (0.204 * d38) + 0.15));
      }
    } else {
      Theta1[0] = 0.0;
      Theta1_2_idx_0 = 0.0;
      Theta2_2_idx_0 = 0.0;
      Theta2_4_idx_0 = 0.0;
      Theta3_2_idx_0 = 0.0;
      Theta3_4_idx_0 = 0.0;
    }
    //     %% FL LEG
    if (legs_on_gnd[1]) {
      double D_FL;
      double D_FL_2;
      b_r_II_c[0] = r_II_c[3] - r_II_B_d[0];
      b_r_II_c[1] = r_II_c[4] - r_II_B_d[1];
      b_r_II_c[2] = r_II_c[5] - r_II_B_d[2];
      coder::mldivide(T_I_B, b_r_II_c, r_BB_c_FL);
      r_B1_c_FL[0] = r_BB_c_FL[0] - 0.12578;
      r_B1_c_FL[1] = r_BB_c_FL[1] - 0.12578;
      r_B1_c_FL[2] = r_BB_c_FL[2] - 0.0254;
      Theta1[1] =
          coder::b_atan2(r_BB_c_FL[1] - 0.12578, r_BB_c_FL[0] - 0.12578) -
          1.5707963267948966;
      dc.re = 0.0;
      dc.im = Theta1[1];
      coder::b_exp(&dc);
      Theta1[1] = coder::angle(dc);
      Theta1_2_idx_1 = Theta1[1] + 3.1415926535897931;
      rotz(Theta1[1], b_dv4);
      coder::mldivide(b_dv4, r_B1_c_FL, r_11_c_FL);
      rotz(Theta1[1] + 3.1415926535897931, b_dv5);
      coder::mldivide(b_dv5, r_B1_c_FL, r_11_c_FL_2);
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
        Theta1_2_idx_1 = 0.0;
        Theta3_4_idx_1 = 0.0;
        Theta2_4_idx_1 = 0.0;
      } else {
        double d3;
        if (coder::fltpower_domain_error(D_FL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d3 = 1.0 - (D_FL_2 * D_FL_2);
        d4 = d3;
        coder::b_sqrt(&d4);
        if (coder::fltpower_domain_error(D_FL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d6 = d3;
        coder::b_sqrt(&d6);
        Theta3_4_idx_1 = coder::b_atan2(-d6, D_FL_2);
        d11 = Theta3_4_idx_1;
        coder::b_sin(&d11);
        d13 = Theta3_4_idx_1;
        coder::b_cos(&d13);
        Theta2_4_idx_1 =
            coder::b_atan2(r_11_c_FL_2[2], r_11_c_FL_2[1] - 0.054) -
            coder::b_atan2(0.204 * d11, (0.204 * d13) + 0.15);
      }
      if (D_FL > 0.999) {
        loop_toggle = false;
        r_II_B_d[0] -= travel_dir_idx_0 * 0.001;
        r_II_B_d[1] -= travel_dir_idx_1 * 0.001;
        r_II_B_d[2] -= travel_dir_idx_2 * 0.001;
        //              r_II_B_d(3) = r_II_B_0(3);
      } else {
        double d7;
        if (coder::fltpower_domain_error(D_FL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d7 = 1.0 - (D_FL * D_FL);
        d9 = d7;
        coder::b_sqrt(&d9);
        Theta3[1] = coder::b_atan2(d9, D_FL);
        if (coder::fltpower_domain_error(D_FL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d15 = d7;
        coder::b_sqrt(&d15);
        Theta3_2_idx_1 = coder::b_atan2(-d15, D_FL);
        d17 = Theta3[1];
        coder::b_sin(&d17);
        d21 = Theta3[1];
        coder::b_cos(&d21);
        d25 = Theta3_2_idx_1;
        coder::b_sin(&d25);
        d29 = Theta3_2_idx_1;
        coder::b_cos(&d29);
        Theta2_2_idx_1 = coder::b_atan2(r_11_c_FL[2], r_11_c_FL[1] - 0.054) -
                         coder::b_atan2(0.204 * d25, (0.204 * d29) + 0.15);
      }
    } else {
      Theta1[1] = 0.0;
      Theta1_2_idx_1 = 0.0;
      Theta2_2_idx_1 = 0.0;
      Theta2_4_idx_1 = 0.0;
      Theta3_2_idx_1 = 0.0;
      Theta3_4_idx_1 = 0.0;
    }
    //     %% BR LEG
    if (legs_on_gnd[2]) {
      double D_BR;
      double D_BR_2;
      double r_BR;
      double r_BR_2;
      double s_BR;
      double s_BR_2;
      b_r_II_c[0] = r_II_c[6] - r_II_B_d[0];
      b_r_II_c[1] = r_II_c[7] - r_II_B_d[1];
      b_r_II_c[2] = r_II_c[8] - r_II_B_d[2];
      coder::mldivide(T_I_B, b_r_II_c, r_BB_c_BR);
      r_B1_c_BR[0] = r_BB_c_BR[0] - -0.12578;
      r_B1_c_BR[1] = r_BB_c_BR[1] - -0.12578;
      r_B1_c_BR[2] = r_BB_c_BR[2] - 0.0254;
      Theta1[2] =
          coder::b_atan2(r_BB_c_BR[1] - -0.12578, r_BB_c_BR[0] - -0.12578) +
          1.5707963267948966;
      dc.re = 0.0;
      dc.im = Theta1[2];
      coder::b_exp(&dc);
      Theta1[2] = coder::angle(dc);
      Theta1_2_idx_2 = Theta1[2] + 3.1415926535897931;
      rotz(Theta1[2], b_dv6);
      coder::mldivide(b_dv6, r_B1_c_BR, b_dv7);
      coder::mldivide(dv10, b_dv7, r_1prime1_c_BR);
      rotz(Theta1[2] + 3.1415926535897931, b_dv8);
      coder::mldivide(b_dv8, r_B1_c_BR, b_dv9);
      coder::mldivide(dv10, b_dv9, r_1prime1_c_BR_2);
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
        Theta1_2_idx_2 = 0.0;
        Theta3_4_idx_2 = 0.0;
        Theta2_4_idx_2 = 0.0;
      } else {
        double Theta3_BR_2_Temp_2;
        double d20;
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d20 = 1.0 - (D_BR_2 * D_BR_2);
        d22 = d20;
        coder::b_sqrt(&d22);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d30 = d20;
        coder::b_sqrt(&d30);
        Theta3_BR_2_Temp_2 = coder::b_atan2(-d30, D_BR_2);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d40 = d20;
        coder::b_sqrt(&d40);
        if (coder::fltpower_domain_error(D_BR_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d44 = d20;
        coder::b_sqrt(&d44);
        Theta3_4_idx_2 = -coder::b_atan2(-d44, D_BR_2);
        d47 = Theta3_BR_2_Temp_2;
        coder::b_sin(&d47);
        d49 = Theta3_BR_2_Temp_2;
        coder::b_cos(&d49);
        Theta2_4_idx_2 = -(coder::b_atan2(s_BR_2, r_BR_2) -
                           coder::b_atan2(0.204 * d47, (0.204 * d49) + 0.15));
      }
      if (D_BR > 0.999) {
        loop_toggle = false;
        r_II_B_d[0] -= travel_dir_idx_0 * 0.001;
        r_II_B_d[1] -= travel_dir_idx_1 * 0.001;
        r_II_B_d[2] -= travel_dir_idx_2 * 0.001;
        //              r_II_B_d(3) = r_II_B_0(3);
      } else {
        double Theta3_BR_2_Temp;
        double Theta3_BR_Temp;
        double d31;
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d31 = 1.0 - (D_BR * D_BR);
        d33 = d31;
        coder::b_sqrt(&d33);
        Theta3_BR_Temp = coder::b_atan2(d33, D_BR);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d41 = d31;
        coder::b_sqrt(&d41);
        Theta3_BR_2_Temp = coder::b_atan2(-d41, D_BR);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d46 = d31;
        coder::b_sqrt(&d46);
        if (coder::fltpower_domain_error(D_BR)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d51 = d31;
        coder::b_sqrt(&d51);
        Theta3_2_idx_2 = -coder::b_atan2(-d51, D_BR);
        d53 = Theta3_BR_Temp;
        coder::b_sin(&d53);
        d54 = Theta3_BR_Temp;
        coder::b_cos(&d54);
        d55 = Theta3_BR_2_Temp;
        coder::b_sin(&d55);
        d56 = Theta3_BR_2_Temp;
        coder::b_cos(&d56);
        Theta2_2_idx_2 = -(coder::b_atan2(s_BR, r_BR) -
                           coder::b_atan2(0.204 * d55, (0.204 * d56) + 0.15));
      }
    } else {
      Theta1[2] = 0.0;
      Theta1_2_idx_2 = 0.0;
      Theta2_2_idx_2 = 0.0;
      Theta2_4_idx_2 = 0.0;
      Theta3_2_idx_2 = 0.0;
      Theta3_4_idx_2 = 0.0;
    }
    //     %% BL LEG
    if (legs_on_gnd[3]) {
      double D_BL;
      double D_BL_2;
      b_r_II_c[0] = r_II_c[9] - r_II_B_d[0];
      b_r_II_c[1] = r_II_c[10] - r_II_B_d[1];
      b_r_II_c[2] = r_II_c[11] - r_II_B_d[2];
      coder::mldivide(T_I_B, b_r_II_c, r_BB_c_BL);
      r_B1_c_BL[0] = r_BB_c_BL[0] - -0.12578;
      r_B1_c_BL[1] = r_BB_c_BL[1] - 0.12578;
      r_B1_c_BL[2] = r_BB_c_BL[2] - 0.0254;
      Theta1[3] =
          coder::b_atan2(r_BB_c_BL[1] - 0.12578, r_BB_c_BL[0] - -0.12578) -
          1.5707963267948966;
      dc.re = 0.0;
      dc.im = Theta1[3];
      coder::b_exp(&dc);
      Theta1[3] = coder::angle(dc);
      Theta1_2_idx_3 = Theta1[3] + 3.1415926535897931;
      rotz(Theta1[3], b_dv10);
      coder::mldivide(b_dv10, r_B1_c_BL, r_11_c_BL);
      rotz(Theta1[3] + 3.1415926535897931, dv11);
      coder::mldivide(dv11, r_B1_c_BL, r_11_c_BL_2);
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
        Theta1_2_idx_3 = 0.0;
        Theta3_4_idx_3 = 0.0;
        Theta2_4_idx_3 = 0.0;
      } else {
        double d24;
        if (coder::fltpower_domain_error(D_BL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d24 = 1.0 - (D_BL_2 * D_BL_2);
        d26 = d24;
        coder::b_sqrt(&d26);
        if (coder::fltpower_domain_error(D_BL_2)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d35 = d24;
        coder::b_sqrt(&d35);
        Theta3_4_idx_3 = coder::b_atan2(-d35, D_BL_2);
        d39 = Theta3_4_idx_3;
        coder::b_sin(&d39);
        d42 = Theta3_4_idx_3;
        coder::b_cos(&d42);
        Theta2_4_idx_3 =
            coder::b_atan2(r_11_c_BL_2[2], r_11_c_BL_2[1] - 0.054) -
            coder::b_atan2(0.204 * d39, (0.204 * d42) + 0.15);
      }
      if (D_BL > 0.999) {
        loop_toggle = false;
        r_II_B_d[0] -= travel_dir_idx_0 * 0.001;
        r_II_B_d[1] -= travel_dir_idx_1 * 0.001;
        r_II_B_d[2] -= travel_dir_idx_2 * 0.001;
        //              r_II_B_d(3) = r_II_B_0(3);
      } else {
        double d36;
        if (coder::fltpower_domain_error(D_BL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d36 = 1.0 - (D_BL * D_BL);
        d37 = d36;
        coder::b_sqrt(&d37);
        Theta3[3] = coder::b_atan2(d37, D_BL);
        if (coder::fltpower_domain_error(D_BL)) {
          c_rtErrorWithMessageID(b_emlrtRTEI.fName, b_emlrtRTEI.lineNo);
        }
        d43 = d36;
        coder::b_sqrt(&d43);
        Theta3_2_idx_3 = coder::b_atan2(-d43, D_BL);
        d45 = Theta3[3];
        coder::b_sin(&d45);
        d48 = Theta3[3];
        coder::b_cos(&d48);
        d50 = Theta3_2_idx_3;
        coder::b_sin(&d50);
        d52 = Theta3_2_idx_3;
        coder::b_cos(&d52);
        Theta2_2_idx_3 = coder::b_atan2(r_11_c_BL[2], r_11_c_BL[1] - 0.054) -
                         coder::b_atan2(0.204 * d50, (0.204 * d52) + 0.15);
      }
    } else {
      Theta1[3] = 0.0;
      Theta1_2_idx_3 = 0.0;
      Theta2_2_idx_3 = 0.0;
      Theta2_4_idx_3 = 0.0;
      Theta3_2_idx_3 = 0.0;
      Theta3_4_idx_3 = 0.0;
    }
  }
  //  if theta1 wraps around into robot
  if ((Theta1[0] <= -1.5707963267948966) || (Theta1[0] >= 3.1415926535897931)) {
    T1_cond_idx_0 = true;
  } else {
    T1_cond_idx_0 = false;
  }
  //  FR
  if ((Theta1[1] <= -3.1415926535897931) || (Theta1[1] >= 1.5707963267948966)) {
    T1_cond_idx_1 = true;
  } else {
    T1_cond_idx_1 = false;
  }
  // FL
  if ((Theta1[2] <= -3.1415926535897931) || (Theta1[2] >= 1.5707963267948966)) {
    T1_cond_idx_2 = true;
  } else {
    T1_cond_idx_2 = false;
  }
  //  BR
  if ((Theta1[3] <= -1.5707963267948966) || (Theta1[3] >= 3.1415926535897931)) {
    T1_cond_idx_3 = true;
  } else {
    T1_cond_idx_3 = false;
  }
  //  BL
  if (T1_cond_idx_0) {
    Theta1[0] = Theta1_2_idx_0;
    Theta2[0] = Theta2_4_idx_0;
    Theta3[0] = Theta3_4_idx_0;
  } else {
    Theta2[0] = Theta2_2_idx_0;
    Theta3[0] = Theta3_2_idx_0;
  }
  if (T1_cond_idx_1) {
    Theta1[1] = Theta1_2_idx_1;
    Theta2[1] = Theta2_4_idx_1;
    Theta3[1] = Theta3_4_idx_1;
  } else {
    Theta2[1] = Theta2_2_idx_1;
    Theta3[1] = Theta3_2_idx_1;
  }
  if (T1_cond_idx_2) {
    Theta1[2] = Theta1_2_idx_2;
    Theta2[2] = Theta2_4_idx_2;
    Theta3[2] = Theta3_4_idx_2;
  } else {
    Theta2[2] = Theta2_2_idx_2;
    Theta3[2] = Theta3_2_idx_2;
  }
  if (T1_cond_idx_3) {
    Theta1[3] = Theta1_2_idx_3;
    Theta2[3] = Theta2_4_idx_3;
    Theta3[3] = Theta3_4_idx_3;
  } else {
    Theta2[3] = Theta2_2_idx_3;
    Theta3[3] = Theta3_2_idx_3;
  }
}

} // namespace Codegen

//
// File trailer for Body_Pose_Controller.cpp
//
// [EOF]
//
