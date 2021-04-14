//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: recursiveKin.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "recursiveKin.h"
#include "Robot_Control_data.h"
#include "skew.h"

// Function Definitions
//
// This function is to be used for recursive calculations of analytical
//  expressions to be used for manipulator kinematics
//  By Nicholas Rossi
//
// Arguments    : const double dotgamma[18]
//                const double ITn[9]
//                const double nTN[9]
//                const double nnrN[3]
//                const double IN_hat[54]
//                const double IN_tilde[54]
//                const double Jn[108]
//                const double dotJn[108]
//                double ITN[9]
//                double NNwI[3]
//                double dotIIrN[3]
//                double JN[108]
//                double dotJN[108]
// Return Type  : void
//
namespace Codegen {
void recursiveKin(const double dotgamma[18], const double ITn[9],
                  const double nTN[9], const double nnrN[3],
                  const double IN_hat[54], const double IN_tilde[54],
                  const double Jn[108], const double dotJn[108], double ITN[9],
                  double NNwI[3], double dotIIrN[3], double JN[108],
                  double dotJN[108])
{
  double b_dv4[108];
  double b_dv5[108];
  double e_a_tmp[108];
  double y[108];
  double b_ITn[54];
  double b_y[54];
  double b_a_tmp[36];
  double b_dv2[36];
  double JN_tmp_tmp[9];
  double a_tmp[9];
  double b_dotJN_tmp[9];
  double b_dv[9];
  double b_dv1[9];
  double b_dv3[9];
  double c_ITn[9];
  double c_a_tmp[9];
  double d_a_tmp[9];
  double dotJN_tmp[9];
  double b_IN_hat[3];
  double b_IN_tilde[3];
  double b_Jn[3];
  for (int i{0}; i < 3; i++) {
    double d;
    double d1;
    double d2;
    d = ITn[i];
    d1 = ITn[i + 3];
    d2 = ITn[i + 6];
    for (int i3{0}; i3 < 3; i3++) {
      ITN[i + (3 * i3)] = ((d * nTN[3 * i3]) + (d1 * nTN[(3 * i3) + 1])) +
                          (d2 * nTN[(3 * i3) + 2]);
    }
  }
  skew(nnrN, JN_tmp_tmp);
  for (int i1{0}; i1 < 9; i1++) {
    a_tmp[i1] = -ITn[i1];
  }
  for (int i2{0}; i2 < 3; i2++) {
    double d3;
    double d4;
    double d5;
    d3 = a_tmp[i2];
    d4 = a_tmp[i2 + 3];
    d5 = a_tmp[i2 + 6];
    for (int i6{0}; i6 < 3; i6++) {
      double d8;
      int i9;
      i9 = i2 + (3 * i6);
      d8 = nTN[i9];
      d_a_tmp[i6 + (3 * i2)] = d8;
      c_a_tmp[i9] =
          ((d3 * JN_tmp_tmp[3 * i6]) + (d4 * JN_tmp_tmp[(3 * i6) + 1])) +
          (d5 * JN_tmp_tmp[(3 * i6) + 2]);
      b_a_tmp[i6 + (6 * i2)] = d8;
      b_a_tmp[i6 + (6 * (i2 + 3))] = 0.0;
    }
  }
  for (int i4{0}; i4 < 3; i4++) {
    int a_tmp_tmp;
    int b_a_tmp_tmp;
    int c_a_tmp_tmp;
    b_a_tmp[(6 * i4) + 3] = c_a_tmp[3 * i4];
    a_tmp_tmp = 6 * (i4 + 3);
    b_a_tmp[a_tmp_tmp + 3] = static_cast<double>(iv[3 * i4]);
    b_a_tmp_tmp = (3 * i4) + 1;
    b_a_tmp[(6 * i4) + 4] = c_a_tmp[b_a_tmp_tmp];
    b_a_tmp[a_tmp_tmp + 4] = static_cast<double>(iv[b_a_tmp_tmp]);
    c_a_tmp_tmp = (3 * i4) + 2;
    b_a_tmp[(6 * i4) + 5] = c_a_tmp[c_a_tmp_tmp];
    b_a_tmp[a_tmp_tmp + 5] = static_cast<double>(iv[c_a_tmp_tmp]);
  }
  for (int i5{0}; i5 < 6; i5++) {
    for (int i8{0}; i8 < 18; i8++) {
      double d7;
      d7 = 0.0;
      for (int i11{0}; i11 < 6; i11++) {
        d7 += b_a_tmp[i5 + (6 * i11)] * Jn[i11 + (6 * i8)];
      }
      y[i5 + (6 * i8)] = d7;
    }
  }
  for (int i7{0}; i7 < 3; i7++) {
    double d10;
    double d6;
    double d9;
    d6 = ITn[i7];
    d9 = ITn[i7 + 3];
    d10 = ITn[i7 + 6];
    for (int i13{0}; i13 < 18; i13++) {
      b_y[i7 + (3 * i13)] =
          y[(i7 + (6 * i13)) + 3] +
          (((d6 * IN_tilde[3 * i13]) + (d9 * IN_tilde[(3 * i13) + 1])) +
           (d10 * IN_tilde[(3 * i13) + 2]));
    }
  }
  for (int i10{0}; i10 < 18; i10++) {
    int JN_tmp;
    int b_JN_tmp;
    int c_JN_tmp;
    int d_JN_tmp;
    JN[6 * i10] = y[6 * i10] + IN_hat[3 * i10];
    JN[(6 * i10) + 3] = b_y[3 * i10];
    JN_tmp = (6 * i10) + 1;
    b_JN_tmp = (3 * i10) + 1;
    JN[JN_tmp] = y[JN_tmp] + IN_hat[b_JN_tmp];
    JN[(6 * i10) + 4] = b_y[b_JN_tmp];
    c_JN_tmp = (6 * i10) + 2;
    d_JN_tmp = (3 * i10) + 2;
    JN[c_JN_tmp] = y[c_JN_tmp] + IN_hat[d_JN_tmp];
    JN[(6 * i10) + 5] = b_y[d_JN_tmp];
  }
  for (int i12{0}; i12 < 3; i12++) {
    double d11;
    double d12;
    double d13;
    double d14;
    double d15;
    d11 = 0.0;
    d12 = 0.0;
    d13 = 0.0;
    d14 = 0.0;
    d15 = 0.0;
    for (int i16{0}; i16 < 18; i16++) {
      double d18;
      int i18;
      int i23;
      i18 = i12 + (6 * i16);
      d18 = dotgamma[i16];
      d11 += JN[i18] * d18;
      d12 += JN[i18 + 3] * d18;
      d13 += Jn[i18] * d18;
      i23 = i12 + (3 * i16);
      d14 += IN_hat[i23] * d18;
      d15 += IN_tilde[i23] * d18;
    }
    b_IN_tilde[i12] = d15;
    b_IN_hat[i12] = d14;
    b_Jn[i12] = d13;
    dotIIrN[i12] = d12;
    NNwI[i12] = d11;
  }
  skew(b_Jn, dotJN_tmp);
  skew(b_IN_hat, b_dv);
  skew(b_IN_tilde, b_dv1);
  for (int i14{0}; i14 < 9; i14++) {
    b_dv[i14] = -b_dv[i14];
  }
  for (int i15{0}; i15 < 3; i15++) {
    double d16;
    double d17;
    double d20;
    double d22;
    double d24;
    double d25;
    d16 = b_dv[i15];
    d17 = b_dv[i15 + 3];
    d20 = b_dv[i15 + 6];
    d22 = dotJN_tmp[i15];
    d24 = dotJN_tmp[i15 + 3];
    d25 = dotJN_tmp[i15 + 6];
    for (int i25{0}; i25 < 3; i25++) {
      int i27;
      int i29;
      int i32;
      i27 = (3 * i25) + 1;
      i29 = (3 * i25) + 2;
      i32 = i15 + (3 * i25);
      b_dv3[i32] = ((d16 * d_a_tmp[3 * i25]) + (d17 * d_a_tmp[i27])) +
                   (d20 * d_a_tmp[i29]);
      b_dotJN_tmp[i32] =
          (((d22 * JN_tmp_tmp[3 * i25]) + (d24 * JN_tmp_tmp[i27])) +
           (d25 * JN_tmp_tmp[i29])) +
          b_dv1[i32];
    }
  }
  for (int i17{0}; i17 < 3; i17++) {
    double d19;
    double d21;
    double d23;
    d19 = a_tmp[i17];
    d21 = a_tmp[i17 + 3];
    d23 = a_tmp[i17 + 6];
    for (int i22{0}; i22 < 3; i22++) {
      c_a_tmp[i17 + (3 * i22)] =
          ((d19 * b_dotJN_tmp[3 * i22]) + (d21 * b_dotJN_tmp[(3 * i22) + 1])) +
          (d23 * b_dotJN_tmp[(3 * i22) + 2]);
      b_dv2[i22 + (6 * i17)] = b_dv3[i22 + (3 * i17)];
      b_dv2[i22 + (6 * (i17 + 3))] = 0.0;
    }
  }
  for (int i19{0}; i19 < 3; i19++) {
    int i21;
    b_dv2[(6 * i19) + 3] = c_a_tmp[3 * i19];
    i21 = 6 * (i19 + 3);
    b_dv2[i21 + 3] = 0.0;
    b_dv2[(6 * i19) + 4] = c_a_tmp[(3 * i19) + 1];
    b_dv2[i21 + 4] = 0.0;
    b_dv2[(6 * i19) + 5] = c_a_tmp[(3 * i19) + 2];
    b_dv2[i21 + 5] = 0.0;
  }
  for (int i20{0}; i20 < 3; i20++) {
    double d26;
    double d27;
    double d28;
    d26 = a_tmp[i20];
    d27 = a_tmp[i20 + 3];
    d28 = a_tmp[i20 + 6];
    for (int i28{0}; i28 < 3; i28++) {
      c_a_tmp[i20 + (3 * i28)] =
          ((d26 * JN_tmp_tmp[3 * i28]) + (d27 * JN_tmp_tmp[(3 * i28) + 1])) +
          (d28 * JN_tmp_tmp[(3 * i28) + 2]);
      b_a_tmp[i28 + (6 * i20)] = d_a_tmp[i28 + (3 * i20)];
      b_a_tmp[i28 + (6 * (i20 + 3))] = 0.0;
    }
  }
  for (int i24{0}; i24 < 3; i24++) {
    int d_a_tmp_tmp;
    int e_a_tmp_tmp;
    int f_a_tmp_tmp;
    b_a_tmp[(6 * i24) + 3] = c_a_tmp[3 * i24];
    d_a_tmp_tmp = 6 * (i24 + 3);
    b_a_tmp[d_a_tmp_tmp + 3] = static_cast<double>(iv[3 * i24]);
    e_a_tmp_tmp = (3 * i24) + 1;
    b_a_tmp[(6 * i24) + 4] = c_a_tmp[e_a_tmp_tmp];
    b_a_tmp[d_a_tmp_tmp + 4] = static_cast<double>(iv[e_a_tmp_tmp]);
    f_a_tmp_tmp = (3 * i24) + 2;
    b_a_tmp[(6 * i24) + 5] = c_a_tmp[f_a_tmp_tmp];
    b_a_tmp[d_a_tmp_tmp + 5] = static_cast<double>(iv[f_a_tmp_tmp]);
  }
  for (int i26{0}; i26 < 6; i26++) {
    for (int i31{0}; i31 < 18; i31++) {
      double d30;
      double d32;
      d30 = 0.0;
      d32 = 0.0;
      for (int i35{0}; i35 < 6; i35++) {
        int i37;
        int i38;
        i37 = i26 + (6 * i35);
        i38 = i35 + (6 * i31);
        d30 += b_dv2[i37] * Jn[i38];
        d32 += b_a_tmp[i37] * dotJn[i38];
      }
      int g_a_tmp_tmp;
      g_a_tmp_tmp = i26 + (6 * i31);
      e_a_tmp[g_a_tmp_tmp] = d32;
      b_dv5[g_a_tmp_tmp] = d30;
    }
  }
  for (int i30{0}; i30 < 3; i30++) {
    double d29;
    double d31;
    double d33;
    double d34;
    double d35;
    double d36;
    d29 = ITn[i30];
    d31 = ITn[i30 + 3];
    d33 = ITn[i30 + 6];
    for (int i36{0}; i36 < 3; i36++) {
      c_ITn[i30 + (3 * i36)] =
          ((d29 * dotJN_tmp[3 * i36]) + (d31 * dotJN_tmp[(3 * i36) + 1])) +
          (d33 * dotJN_tmp[(3 * i36) + 2]);
    }
    d34 = c_ITn[i30];
    d35 = c_ITn[i30 + 3];
    d36 = c_ITn[i30 + 6];
    for (int i39{0}; i39 < 18; i39++) {
      b_ITn[i30 + (3 * i39)] =
          ((d34 * IN_tilde[3 * i39]) + (d35 * IN_tilde[(3 * i39) + 1])) +
          (d36 * IN_tilde[(3 * i39) + 2]);
    }
  }
  for (int i33{0}; i33 < 18; i33++) {
    b_dv4[6 * i33] = 0.0;
    b_dv4[(6 * i33) + 3] = b_ITn[3 * i33];
    b_dv4[(6 * i33) + 1] = 0.0;
    b_dv4[(6 * i33) + 4] = b_ITn[(3 * i33) + 1];
    b_dv4[(6 * i33) + 2] = 0.0;
    b_dv4[(6 * i33) + 5] = b_ITn[(3 * i33) + 2];
  }
  for (int i34{0}; i34 < 108; i34++) {
    dotJN[i34] = (b_dv5[i34] + e_a_tmp[i34]) + b_dv4[i34];
  }
}

} // namespace Codegen

//
// File trailer for recursiveKin.cpp
//
// [EOF]
//
