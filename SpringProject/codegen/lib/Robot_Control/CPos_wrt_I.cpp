//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CPos_wrt_I.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "CPos_wrt_I.h"
#include "Robot_Control_data.h"
#include "rotx.h"
#include "rotz.h"

// Function Definitions
//
// This function gives the positions of the contact points of the robot with
//  respect to the inertial frame.
//  jnt_var = [FR_vars,FL_vars,BR_vars,BL_vars]
//  FR_vars = [Theta1(1),Theta2(1),Theta3(1)];
//  FL_vars = [Theta1(2),Theta2(2),Theta3(2)];
//  BR_vars = [Theta1(3),Theta2(3),Theta3(3)];
//  BL_vars = [Theta1(4),Theta2(4),Theta3(4)];
//  BodyRot = zyx rotation of body
//  rBfromI = vector from I to B wrt I.
//
// Arguments    : const double Theta1[4]
//                const double Theta2[4]
//                const double Theta3[4]
//                const double TB[9]
//                const double rBfromI[3]
//                double rc_FR[3]
//                double rc_FL[3]
//                double rc_BR[3]
//                double rc_BL[3]
// Return Type  : void
//
namespace Codegen {
void CPos_wrt_I(const double Theta1[4], const double Theta2[4],
                const double Theta3[4], const double TB[9],
                const double rBfromI[3], double rc_FR[3], double rc_FL[3],
                double rc_BR[3], double rc_BL[3])
{
  double T1_BL[9];
  double T1_BR[9];
  double T1_FL[9];
  double T1_FR[9];
  double T2_BL[9];
  double T2_BR[9];
  double T2_FL[9];
  double T2_FR[9];
  double b_dv[9];
  double b_rBfromI[3];
  //  Relative Positions
  //  Orientations wrt I
  //  Orientation of Body
  // TB = rotz(BodyRot(1))*roty(BodyRot(2))*rotx(BodyRot(3));
  //  Orientation of FR leg
  rotz(Theta1[0], b_dv);
  for (int i{0}; i < 3; i++) {
    double d;
    double d1;
    double d2;
    d = TB[i];
    d1 = TB[i + 3];
    d2 = TB[i + 6];
    for (int i2{0}; i2 < 3; i2++) {
      T1_FR[i + (3 * i2)] = ((d * b_dv[3 * i2]) + (d1 * b_dv[(3 * i2) + 1])) +
                            (d2 * b_dv[(3 * i2) + 2]);
    }
  }
  rotx(Theta2[0], b_dv);
  for (int i1{0}; i1 < 3; i1++) {
    double d3;
    double d4;
    double d5;
    d3 = T1_FR[i1];
    d4 = T1_FR[i1 + 3];
    d5 = T1_FR[i1 + 6];
    for (int i4{0}; i4 < 3; i4++) {
      T2_FR[i1 + (3 * i4)] = ((d3 * b_dv[3 * i4]) + (d4 * b_dv[(3 * i4) + 1])) +
                             (d5 * b_dv[(3 * i4) + 2]);
    }
  }
  //  Orientation of FR leg
  rotz(Theta1[1], b_dv);
  for (int i3{0}; i3 < 3; i3++) {
    double d6;
    double d7;
    double d8;
    d6 = TB[i3];
    d7 = TB[i3 + 3];
    d8 = TB[i3 + 6];
    for (int i6{0}; i6 < 3; i6++) {
      T1_FL[i3 + (3 * i6)] = ((d6 * b_dv[3 * i6]) + (d7 * b_dv[(3 * i6) + 1])) +
                             (d8 * b_dv[(3 * i6) + 2]);
    }
  }
  rotx(Theta2[1], b_dv);
  for (int i5{0}; i5 < 3; i5++) {
    double d10;
    double d11;
    double d9;
    d9 = T1_FL[i5];
    d10 = T1_FL[i5 + 3];
    d11 = T1_FL[i5 + 6];
    for (int i8{0}; i8 < 3; i8++) {
      T2_FL[i5 + (3 * i8)] =
          ((d9 * b_dv[3 * i8]) + (d10 * b_dv[(3 * i8) + 1])) +
          (d11 * b_dv[(3 * i8) + 2]);
    }
  }
  //  Orientation of BR leg
  rotz(Theta1[2], b_dv);
  for (int i7{0}; i7 < 3; i7++) {
    double d12;
    double d13;
    double d14;
    d12 = TB[i7];
    d13 = TB[i7 + 3];
    d14 = TB[i7 + 6];
    for (int i10{0}; i10 < 3; i10++) {
      T1_BR[i7 + (3 * i10)] =
          ((d12 * b_dv[3 * i10]) + (d13 * b_dv[(3 * i10) + 1])) +
          (d14 * b_dv[(3 * i10) + 2]);
    }
  }
  rotx(Theta2[2], b_dv);
  for (int i9{0}; i9 < 3; i9++) {
    double d15;
    double d16;
    double d17;
    d15 = T1_BR[i9];
    d16 = T1_BR[i9 + 3];
    d17 = T1_BR[i9 + 6];
    for (int i12{0}; i12 < 3; i12++) {
      T2_BR[i9 + (3 * i12)] =
          ((d15 * b_dv[3 * i12]) + (d16 * b_dv[(3 * i12) + 1])) +
          (d17 * b_dv[(3 * i12) + 2]);
    }
  }
  //  Orientation of BL leg
  rotz(Theta1[3], b_dv);
  for (int i11{0}; i11 < 3; i11++) {
    double d18;
    double d19;
    double d20;
    d18 = TB[i11];
    d19 = TB[i11 + 3];
    d20 = TB[i11 + 6];
    for (int i14{0}; i14 < 3; i14++) {
      T1_BL[i11 + (3 * i14)] =
          ((d18 * b_dv[3 * i14]) + (d19 * b_dv[(3 * i14) + 1])) +
          (d20 * b_dv[(3 * i14) + 2]);
    }
  }
  rotx(Theta2[3], b_dv);
  for (int i13{0}; i13 < 3; i13++) {
    double d21;
    double d22;
    double d23;
    d21 = T1_BL[i13];
    d22 = T1_BL[i13 + 3];
    d23 = T1_BL[i13 + 6];
    for (int i15{0}; i15 < 3; i15++) {
      T2_BL[i13 + (3 * i15)] =
          ((d21 * b_dv[3 * i15]) + (d22 * b_dv[(3 * i15) + 1])) +
          (d23 * b_dv[(3 * i15) + 2]);
    }
  }
  //  Positions wrt I
  //  Positions of FR Leg
  rotx(Theta3[0], b_dv);
  //  Positions of FL Leg
  for (int i16{0}; i16 < 3; i16++) {
    double d24;
    double d25;
    double d26;
    double d27;
    double d29;
    d24 = 0.0;
    d25 = 0.0;
    d26 = 0.0;
    d27 = 0.0;
    d29 = 0.0;
    for (int i18{0}; i18 < 3; i18++) {
      double d35;
      int i20;
      i20 = i16 + (3 * i18);
      d35 = TB[i20];
      d24 += d35 * dv[i18];
      d27 += T1_FR[i20] * dv1[i18];
      d25 += T2_FR[i20] * dv2[i18];
      d29 += (((T2_FR[i16] * b_dv[3 * i18]) +
               (T2_FR[i16 + 3] * b_dv[(3 * i18) + 1])) +
              (T2_FR[i16 + 6] * b_dv[(3 * i18) + 2])) *
             dv3[i18];
      d26 += d35 * dv4[i18];
    }
    double d33;
    d33 = rBfromI[i16];
    rc_FR[i16] = (((d33 + d24) + d27) + d25) + d29;
    b_rBfromI[i16] = d33 + d26;
  }
  rotx(Theta3[1], b_dv);
  //  Position of BR Leg
  for (int i17{0}; i17 < 3; i17++) {
    double d28;
    double d30;
    double d31;
    double d32;
    d28 = 0.0;
    d30 = 0.0;
    d31 = 0.0;
    d32 = 0.0;
    for (int i21{0}; i21 < 3; i21++) {
      int T1_FL_tmp;
      T1_FL_tmp = i17 + (3 * i21);
      d31 += T1_FL[T1_FL_tmp] * dv5[i21];
      d28 += T2_FL[T1_FL_tmp] * dv6[i21];
      d32 += (((T2_FL[i17] * b_dv[3 * i21]) +
               (T2_FL[i17 + 3] * b_dv[(3 * i21) + 1])) +
              (T2_FL[i17 + 6] * b_dv[(3 * i21) + 2])) *
             dv7[i21];
      d30 += TB[T1_FL_tmp] * dv8[i21];
    }
    rc_FL[i17] = ((b_rBfromI[i17] + d31) + d28) + d32;
    b_rBfromI[i17] = rBfromI[i17] + d30;
  }
  rotx(Theta3[2], b_dv);
  //  Position of BL Leg
  for (int i19{0}; i19 < 3; i19++) {
    double d34;
    double d36;
    double d37;
    double d39;
    d34 = 0.0;
    d36 = 0.0;
    d37 = 0.0;
    d39 = 0.0;
    for (int i23{0}; i23 < 3; i23++) {
      int T1_BR_tmp;
      T1_BR_tmp = i19 + (3 * i23);
      d37 += T1_BR[T1_BR_tmp] * dv1[i23];
      d34 += T2_BR[T1_BR_tmp] * dv2[i23];
      d39 += (((T2_BR[i19] * b_dv[3 * i23]) +
               (T2_BR[i19 + 3] * b_dv[(3 * i23) + 1])) +
              (T2_BR[i19 + 6] * b_dv[(3 * i23) + 2])) *
             dv3[i23];
      d36 += TB[T1_BR_tmp] * dv9[i23];
    }
    rc_BR[i19] = ((b_rBfromI[i19] + d37) + d34) + d39;
    b_rBfromI[i19] = rBfromI[i19] + d36;
  }
  rotx(Theta3[3], b_dv);
  for (int i22{0}; i22 < 3; i22++) {
    double d38;
    double d40;
    double d41;
    d38 = 0.0;
    d40 = 0.0;
    d41 = 0.0;
    for (int i24{0}; i24 < 3; i24++) {
      int T1_BL_tmp;
      T1_BL_tmp = i22 + (3 * i24);
      d40 += T1_BL[T1_BL_tmp] * dv5[i24];
      d38 += T2_BL[T1_BL_tmp] * dv6[i24];
      d41 += (((T2_BL[i22] * b_dv[3 * i24]) +
               (T2_BL[i22 + 3] * b_dv[(3 * i24) + 1])) +
              (T2_BL[i22 + 6] * b_dv[(3 * i24) + 2])) *
             dv7[i24];
    }
    rc_BL[i22] = ((b_rBfromI[i22] + d40) + d38) + d41;
  }
}

} // namespace Codegen

//
// File trailer for CPos_wrt_I.cpp
//
// [EOF]
//
