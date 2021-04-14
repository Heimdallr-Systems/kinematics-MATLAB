//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: contactJacobians.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "contactJacobians.h"
#include "Robot_Control_data.h"
#include "cos.h"
#include "recursiveKin.h"
#include "rotx.h"
#include "roty.h"
#include "rotz.h"
#include "sin.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double state[36]
//                double GeoJc_FR[108]
//                double GeoJc_FL[108]
//                double GeoJc_BR[108]
//                double GeoJc_BL[108]
// Return Type  : void
//
namespace Codegen {
void contactJacobians(const double state[36], double GeoJc_FR[108],
                      double GeoJc_FL[108], double GeoJc_BR[108],
                      double GeoJc_BL[108])
{
  double GeoJ1_BL[108];
  double GeoJ1_BR[108];
  double GeoJ1_FL[108];
  double GeoJ1_FR[108];
  double GeoJ2_BL[108];
  double GeoJ2_BR[108];
  double GeoJ2_FL[108];
  double GeoJ2_FR[108];
  double GeoJ3_BL[108];
  double GeoJ3_BR[108];
  double GeoJ3_FL[108];
  double GeoJ3_FR[108];
  double GeoJB[108];
  double a__10[108];
  double a__20[108];
  double a__30[108];
  double a__40[108];
  double dotGeoJ1_BL[108];
  double dotGeoJ1_BR[108];
  double dotGeoJ1_FL[108];
  double dotGeoJ1_FR[108];
  double dotGeoJ2_BL[108];
  double dotGeoJ2_BR[108];
  double dotGeoJ2_FL[108];
  double dotGeoJ2_FR[108];
  double dotGeoJ3_BL[108];
  double dotGeoJ3_BR[108];
  double dotGeoJ3_FL[108];
  double dotGeoJ3_FR[108];
  double dotGeoJB[108];
  double I1_hat_BL[54];
  double I1_hat_BR[54];
  double I1_hat_FL[54];
  double I1_hat_FR[54];
  double I2_hat_BL[54];
  double I2_hat_BR[54];
  double I2_hat_FL[54];
  double I2_hat_FR[54];
  double I3_hat_BL[54];
  double I3_hat_BR[54];
  double I3_hat_FL[54];
  double I3_hat_FR[54];
  double b_dv4[54];
  double b_dv7[54];
  double T_2_3_BL[9];
  double T_2_3_BR[9];
  double T_2_3_FL[9];
  double T_2_3_FR[9];
  double T_I_1_BL[9];
  double T_I_1_BR[9];
  double T_I_1_FL[9];
  double T_I_1_FR[9];
  double T_I_2_BL[9];
  double T_I_2_BR[9];
  double T_I_2_FL[9];
  double T_I_2_FR[9];
  double T_I_3_BL[9];
  double T_I_3_BR[9];
  double T_I_3_FL[9];
  double T_I_3_FR[9];
  double T_I_B[9];
  double a__17[9];
  double a__27[9];
  double a__37[9];
  double a__7[9];
  double b_dv[9];
  double b_dv1[9];
  double b_dv10[9];
  double b_dv2[9];
  double b_dv3[9];
  double b_dv5[9];
  double b_dv6[9];
  double b_dv8[9];
  double b_dv9[9];
  double dv11[9];
  double dv12[9];
  double dv13[9];
  double a__1[3];
  double a__11[3];
  double a__12[3];
  double a__13[3];
  double a__14[3];
  double a__15[3];
  double a__16[3];
  double a__18[3];
  double a__19[3];
  double a__2[3];
  double a__21[3];
  double a__22[3];
  double a__23[3];
  double a__24[3];
  double a__25[3];
  double a__26[3];
  double a__28[3];
  double a__29[3];
  double a__3[3];
  double a__31[3];
  double a__32[3];
  double a__33[3];
  double a__34[3];
  double a__35[3];
  double a__36[3];
  double a__38[3];
  double a__39[3];
  double a__4[3];
  double a__5[3];
  double a__6[3];
  double a__8[3];
  double a__9[3];
  double GeoJB_tmp;
  double b_GeoJB_tmp;
  double c_GeoJB_tmp;
  double d_GeoJB_tmp;
  //  Initialize variables
  (void)std::memset(&GeoJB[0], 0, 108U * (sizeof(double)));
  (void)std::memset(&dotGeoJB[0], 0, 108U * (sizeof(double)));
  //  Relative Orientations (T_(reference)_(new))
  rotz(state[0], b_dv);
  roty(state[1], b_dv1);
  rotx(state[2], b_dv2);
  for (int i{0}; i < 3; i++) {
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    d = b_dv[i];
    d1 = b_dv[i + 3];
    d2 = b_dv[i + 6];
    for (int i1{0}; i1 < 3; i1++) {
      b_dv3[i + (3 * i1)] = ((d * b_dv1[3 * i1]) + (d1 * b_dv1[(3 * i1) + 1])) +
                            (d2 * b_dv1[(3 * i1) + 2]);
    }
    d3 = b_dv3[i];
    d4 = b_dv3[i + 3];
    d5 = b_dv3[i + 6];
    for (int i2{0}; i2 < 3; i2++) {
      T_I_B[i + (3 * i2)] =
          ((d3 * b_dv2[3 * i2]) + (d4 * b_dv2[(3 * i2) + 1])) +
          (d5 * b_dv2[(3 * i2) + 2]);
    }
  }
  rotx(state[14], T_2_3_FR);
  rotx(state[15], T_2_3_FL);
  rotx(state[16], T_2_3_BR);
  rotx(state[17], T_2_3_BL);
  //  phi = state(1);
  //  dotphi = state(19);
  //  Set Geometric Jacobian
  GeoJB_tmp = state[1];
  coder::b_sin(&GeoJB_tmp);
  GeoJB[0] = -GeoJB_tmp;
  GeoJB[12] = 1.0;
  b_GeoJB_tmp = state[1];
  coder::b_cos(&b_GeoJB_tmp);
  c_GeoJB_tmp = state[2];
  coder::b_sin(&c_GeoJB_tmp);
  GeoJB[1] = b_GeoJB_tmp * c_GeoJB_tmp;
  d_GeoJB_tmp = state[2];
  coder::b_cos(&d_GeoJB_tmp);
  GeoJB[7] = d_GeoJB_tmp;
  GeoJB[2] = d_GeoJB_tmp * b_GeoJB_tmp;
  GeoJB[8] = -c_GeoJB_tmp;
  GeoJB[21] = 1.0;
  GeoJB[28] = 1.0;
  GeoJB[35] = 1.0;
  //  Set time derivitive of Geometric Jacobian
  dotGeoJB[0] = (-state[19]) * b_GeoJB_tmp;
  dotGeoJB[1] = ((state[20] * d_GeoJB_tmp) * b_GeoJB_tmp) -
                ((state[19] * c_GeoJB_tmp) * GeoJB_tmp);
  dotGeoJB[7] = (-state[20]) * c_GeoJB_tmp;
  dotGeoJB[2] = (((-state[20]) * b_GeoJB_tmp) * c_GeoJB_tmp) -
                ((state[19] * d_GeoJB_tmp) * GeoJB_tmp);
  dotGeoJB[8] = (-state[20]) * d_GeoJB_tmp;
  (void)std::memset(&I1_hat_FR[0], 0, 54U * (sizeof(double)));
  I1_hat_FR[20] = 1.0;
  //  z-axis, Theta1_FR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotz(state[6], b_dv5);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_B, b_dv5, dv, I1_hat_FR,
               b_dv4, GeoJB, dotGeoJB, T_I_1_FR, a__1, a__2, GeoJ1_FR,
               dotGeoJ1_FR);
  (void)std::memset(&I2_hat_FR[0], 0, 54U * (sizeof(double)));
  I2_hat_FR[30] = 1.0;
  //  x-axis, Theta2_FR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotx(state[10], b_dv6);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_1_FR, b_dv6, dv1, I2_hat_FR,
               b_dv4, GeoJ1_FR, dotGeoJ1_FR, T_I_2_FR, a__3, a__4, GeoJ2_FR,
               dotGeoJ2_FR);
  (void)std::memset(&I3_hat_FR[0], 0, 54U * (sizeof(double)));
  I3_hat_FR[42] = 1.0;
  //  x-axis, Theta3_FR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_2_FR, T_2_3_FR, dv2,
               I3_hat_FR, b_dv4, GeoJ2_FR, dotGeoJ2_FR, T_I_3_FR, a__5, a__6,
               GeoJ3_FR, dotGeoJ3_FR);
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  (void)std::memset(&b_dv7[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_3_FR, T_2_3_FR, dv3, b_dv4,
               b_dv7, GeoJ3_FR, dotGeoJ3_FR, a__7, a__8, a__9, GeoJc_FR, a__10);
  (void)std::memset(&I1_hat_FL[0], 0, 54U * (sizeof(double)));
  I1_hat_FL[23] = 1.0;
  //  z-axis, Theta1_FL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotz(state[7], b_dv8);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_B, b_dv8, dv4, I1_hat_FL,
               b_dv4, GeoJB, dotGeoJB, T_I_1_FL, a__11, a__12, GeoJ1_FL,
               dotGeoJ1_FL);
  (void)std::memset(&I2_hat_FL[0], 0, 54U * (sizeof(double)));
  I2_hat_FL[33] = 1.0;
  //  x-axis, Theta2_FL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotx(state[11], b_dv9);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_1_FL, b_dv9, dv5, I2_hat_FL,
               b_dv4, GeoJ1_FL, dotGeoJ1_FL, T_I_2_FL, a__13, a__14, GeoJ2_FL,
               dotGeoJ2_FL);
  (void)std::memset(&I3_hat_FL[0], 0, 54U * (sizeof(double)));
  I3_hat_FL[45] = 1.0;
  //  x-axis, Theta3_FL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_2_FL, T_2_3_FL, dv6,
               I3_hat_FL, b_dv4, GeoJ2_FL, dotGeoJ2_FL, T_I_3_FL, a__15, a__16,
               GeoJ3_FL, dotGeoJ3_FL);
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  (void)std::memset(&b_dv7[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_3_FL, T_2_3_FL, dv7, b_dv4,
               b_dv7, GeoJ3_FL, dotGeoJ3_FL, a__17, a__18, a__19, GeoJc_FL,
               a__20);
  (void)std::memset(&I1_hat_BR[0], 0, 54U * (sizeof(double)));
  I1_hat_BR[26] = 1.0;
  //  z-axis, Theta1_BR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotz(state[8], b_dv10);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_B, b_dv10, dv8, I1_hat_BR,
               b_dv4, GeoJB, dotGeoJB, T_I_1_BR, a__21, a__22, GeoJ1_BR,
               dotGeoJ1_BR);
  (void)std::memset(&I2_hat_BR[0], 0, 54U * (sizeof(double)));
  I2_hat_BR[36] = 1.0;
  //  x-axis, Theta2_BR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotx(state[12], dv11);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_1_BR, dv11, dv1, I2_hat_BR,
               b_dv4, GeoJ1_BR, dotGeoJ1_BR, T_I_2_BR, a__23, a__24, GeoJ2_BR,
               dotGeoJ2_BR);
  (void)std::memset(&I3_hat_BR[0], 0, 54U * (sizeof(double)));
  I3_hat_BR[48] = 1.0;
  //  x-axis, Theta3_BR
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_2_BR, T_2_3_BR, dv2,
               I3_hat_BR, b_dv4, GeoJ2_BR, dotGeoJ2_BR, T_I_3_BR, a__25, a__26,
               GeoJ3_BR, dotGeoJ3_BR);
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  (void)std::memset(&b_dv7[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_3_BR, T_2_3_BR, dv3, b_dv4,
               b_dv7, GeoJ3_BR, dotGeoJ3_BR, a__27, a__28, a__29, GeoJc_BR,
               a__30);
  (void)std::memset(&I1_hat_BL[0], 0, 54U * (sizeof(double)));
  I1_hat_BL[29] = 1.0;
  //  z-axis, Theta1_BL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotz(state[9], dv12);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_B, dv12, dv9, I1_hat_BL,
               b_dv4, GeoJB, dotGeoJB, T_I_1_BL, a__31, a__32, GeoJ1_BL,
               dotGeoJ1_BL);
  (void)std::memset(&I2_hat_BL[0], 0, 54U * (sizeof(double)));
  I2_hat_BL[39] = 1.0;
  //  x-axis, Theta2_BL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  rotx(state[13], dv13);
  recursiveKin(*((double(*)[18])(&state[18])), T_I_1_BL, dv13, dv5, I2_hat_BL,
               b_dv4, GeoJ1_BL, dotGeoJ1_BL, T_I_2_BL, a__33, a__34, GeoJ2_BL,
               dotGeoJ2_BL);
  (void)std::memset(&I3_hat_BL[0], 0, 54U * (sizeof(double)));
  I3_hat_BL[51] = 1.0;
  //  x-axis, Theta3_BL
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_2_BL, T_2_3_BL, dv6,
               I3_hat_BL, b_dv4, GeoJ2_BL, dotGeoJ2_BL, T_I_3_BL, a__35, a__36,
               GeoJ3_BL, dotGeoJ3_BL);
  (void)std::memset(&b_dv4[0], 0, 54U * (sizeof(double)));
  (void)std::memset(&b_dv7[0], 0, 54U * (sizeof(double)));
  recursiveKin(*((double(*)[18])(&state[18])), T_I_3_BL, T_2_3_BL, dv7, b_dv4,
               b_dv7, GeoJ3_BL, dotGeoJ3_BL, a__37, a__38, a__39, GeoJc_BL,
               a__40);
}

} // namespace Codegen

//
// File trailer for contactJacobians.cpp
//
// [EOF]
//
