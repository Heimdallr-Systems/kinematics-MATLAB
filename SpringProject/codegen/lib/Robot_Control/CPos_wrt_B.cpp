//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CPos_wrt_B.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "CPos_wrt_B.h"
#include "Robot_Control_data.h"
#include "rotx.h"
#include "rotz.h"

// Function Definitions
//
// This function gives the positions of the contact points of the robot with
//  respect to the body frame.
//  Theta1 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL]
//  Theta2 = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]
//  Theta3 = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]
//
// Arguments    : const double Theta1[4]
//                const double Theta2[4]
//                const double Theta3[4]
//                double rc_FR[3]
//                double rc_FL[3]
//                double rc_BR[3]
//                double rc_BL[3]
// Return Type  : void
//
namespace Codegen {
void CPos_wrt_B(const double Theta1[4], const double Theta2[4],
                const double Theta3[4], double rc_FR[3], double rc_FL[3],
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
  //  Relative Positions
  //  place inertial frame at base
  //  big plate: Short: 350, L: 700;
  //  Relative FR Leg Positions
  //  Orientations wrt I
  //  Orientation of Body
  //  TB = rotx(jnt_var(13))*roty(jnt_var(14))*rotz(jnt_var(15));
  //  Orientation of FR leg
  rotz(Theta1[0], b_dv);
  for (int i{0}; i < 3; i++) {
    signed char i1;
    signed char i3;
    signed char i4;
    i1 = iv[i];
    i3 = iv[i + 3];
    i4 = iv[i + 6];
    for (int i5{0}; i5 < 3; i5++) {
      T1_FR[i + (3 * i5)] = (((static_cast<double>(i1)) * b_dv[3 * i5]) +
                             ((static_cast<double>(i3)) * b_dv[(3 * i5) + 1])) +
                            ((static_cast<double>(i4)) * b_dv[(3 * i5) + 2]);
    }
  }
  rotx(Theta2[0], b_dv);
  for (int i2{0}; i2 < 3; i2++) {
    double d;
    double d1;
    double d2;
    d = T1_FR[i2];
    d1 = T1_FR[i2 + 3];
    d2 = T1_FR[i2 + 6];
    for (int i7{0}; i7 < 3; i7++) {
      T2_FR[i2 + (3 * i7)] = ((d * b_dv[3 * i7]) + (d1 * b_dv[(3 * i7) + 1])) +
                             (d2 * b_dv[(3 * i7) + 2]);
    }
  }
  //  Orientation of FR leg
  rotz(Theta1[1], b_dv);
  for (int i6{0}; i6 < 3; i6++) {
    signed char i10;
    signed char i11;
    signed char i8;
    i8 = iv[i6];
    i10 = iv[i6 + 3];
    i11 = iv[i6 + 6];
    for (int i12{0}; i12 < 3; i12++) {
      T1_FL[i6 + (3 * i12)] =
          (((static_cast<double>(i8)) * b_dv[3 * i12]) +
           ((static_cast<double>(i10)) * b_dv[(3 * i12) + 1])) +
          ((static_cast<double>(i11)) * b_dv[(3 * i12) + 2]);
    }
  }
  rotx(Theta2[1], b_dv);
  for (int i9{0}; i9 < 3; i9++) {
    double d3;
    double d4;
    double d5;
    d3 = T1_FL[i9];
    d4 = T1_FL[i9 + 3];
    d5 = T1_FL[i9 + 6];
    for (int i14{0}; i14 < 3; i14++) {
      T2_FL[i9 + (3 * i14)] =
          ((d3 * b_dv[3 * i14]) + (d4 * b_dv[(3 * i14) + 1])) +
          (d5 * b_dv[(3 * i14) + 2]);
    }
  }
  //  Orientation of BR leg
  rotz(Theta1[2], b_dv);
  for (int i13{0}; i13 < 3; i13++) {
    signed char i15;
    signed char i17;
    signed char i18;
    i15 = iv[i13];
    i17 = iv[i13 + 3];
    i18 = iv[i13 + 6];
    for (int i19{0}; i19 < 3; i19++) {
      T1_BR[i13 + (3 * i19)] =
          (((static_cast<double>(i15)) * b_dv[3 * i19]) +
           ((static_cast<double>(i17)) * b_dv[(3 * i19) + 1])) +
          ((static_cast<double>(i18)) * b_dv[(3 * i19) + 2]);
    }
  }
  rotx(Theta2[2], b_dv);
  for (int i16{0}; i16 < 3; i16++) {
    double d6;
    double d7;
    double d8;
    d6 = T1_BR[i16];
    d7 = T1_BR[i16 + 3];
    d8 = T1_BR[i16 + 6];
    for (int i21{0}; i21 < 3; i21++) {
      T2_BR[i16 + (3 * i21)] =
          ((d6 * b_dv[3 * i21]) + (d7 * b_dv[(3 * i21) + 1])) +
          (d8 * b_dv[(3 * i21) + 2]);
    }
  }
  //  Orientation of BL leg
  rotz(Theta1[3], b_dv);
  for (int i20{0}; i20 < 3; i20++) {
    signed char i22;
    signed char i24;
    signed char i25;
    i22 = iv[i20];
    i24 = iv[i20 + 3];
    i25 = iv[i20 + 6];
    for (int i26{0}; i26 < 3; i26++) {
      T1_BL[i20 + (3 * i26)] =
          (((static_cast<double>(i22)) * b_dv[3 * i26]) +
           ((static_cast<double>(i24)) * b_dv[(3 * i26) + 1])) +
          ((static_cast<double>(i25)) * b_dv[(3 * i26) + 2]);
    }
  }
  rotx(Theta2[3], b_dv);
  for (int i23{0}; i23 < 3; i23++) {
    double d10;
    double d11;
    double d9;
    d9 = T1_BL[i23];
    d10 = T1_BL[i23 + 3];
    d11 = T1_BL[i23 + 6];
    for (int i28{0}; i28 < 3; i28++) {
      T2_BL[i23 + (3 * i28)] =
          ((d9 * b_dv[3 * i28]) + (d10 * b_dv[(3 * i28) + 1])) +
          (d11 * b_dv[(3 * i28) + 2]);
    }
  }
  //  Positions wrt I
  //  Positions of FR Leg
  rotx(Theta3[0], b_dv);
  for (int i27{0}; i27 < 3; i27++) {
    double d12;
    double d13;
    double d14;
    d12 = 0.0;
    d13 = 0.0;
    d14 = 0.0;
    for (int i30{0}; i30 < 3; i30++) {
      int i31;
      i31 = i27 + (3 * i30);
      d12 += T1_FR[i31] * dv1[i30];
      d14 += T2_FR[i31] * dv2[i30];
      d13 += (((T2_FR[i27] * b_dv[3 * i30]) +
               (T2_FR[i27 + 3] * b_dv[(3 * i30) + 1])) +
              (T2_FR[i27 + 6] * b_dv[(3 * i30) + 2])) *
             dv3[i30];
    }
    rc_FR[i27] = ((dv[i27] + d12) + d14) + d13;
  }
  //  Positions of FL Leg
  rotx(Theta3[1], b_dv);
  for (int i29{0}; i29 < 3; i29++) {
    double d15;
    double d16;
    double d17;
    d15 = 0.0;
    d16 = 0.0;
    d17 = 0.0;
    for (int i33{0}; i33 < 3; i33++) {
      int i34;
      i34 = i29 + (3 * i33);
      d15 += T1_FL[i34] * dv5[i33];
      d17 += T2_FL[i34] * dv6[i33];
      d16 += (((T2_FL[i29] * b_dv[3 * i33]) +
               (T2_FL[i29 + 3] * b_dv[(3 * i33) + 1])) +
              (T2_FL[i29 + 6] * b_dv[(3 * i33) + 2])) *
             dv7[i33];
    }
    rc_FL[i29] = ((dv4[i29] + d15) + d17) + d16;
  }
  //  Position of BR Leg
  rotx(Theta3[2], b_dv);
  for (int i32{0}; i32 < 3; i32++) {
    double d18;
    double d19;
    double d20;
    d18 = 0.0;
    d19 = 0.0;
    d20 = 0.0;
    for (int i36{0}; i36 < 3; i36++) {
      int i37;
      i37 = i32 + (3 * i36);
      d18 += T1_BR[i37] * dv1[i36];
      d20 += T2_BR[i37] * dv2[i36];
      d19 += (((T2_BR[i32] * b_dv[3 * i36]) +
               (T2_BR[i32 + 3] * b_dv[(3 * i36) + 1])) +
              (T2_BR[i32 + 6] * b_dv[(3 * i36) + 2])) *
             dv3[i36];
    }
    rc_BR[i32] = ((dv8[i32] + d18) + d20) + d19;
  }
  //  Position of BL Leg
  rotx(Theta3[3], b_dv);
  for (int i35{0}; i35 < 3; i35++) {
    double d21;
    double d22;
    double d23;
    d21 = 0.0;
    d22 = 0.0;
    d23 = 0.0;
    for (int i38{0}; i38 < 3; i38++) {
      int i39;
      i39 = i35 + (3 * i38);
      d21 += T1_BL[i39] * dv5[i38];
      d23 += T2_BL[i39] * dv6[i38];
      d22 += (((T2_BL[i35] * b_dv[3 * i38]) +
               (T2_BL[i35 + 3] * b_dv[(3 * i38) + 1])) +
              (T2_BL[i35 + 6] * b_dv[(3 * i38) + 2])) *
             dv7[i38];
    }
    rc_BL[i35] = ((dv9[i35] + d21) + d23) + d22;
  }
}

} // namespace Codegen

//
// File trailer for CPos_wrt_B.cpp
//
// [EOF]
//
