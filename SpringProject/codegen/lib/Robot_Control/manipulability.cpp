//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: manipulability.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "manipulability.h"
#include "contactJacobians.h"
#include "prod.h"
#include "svd.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double state[36]
//                double *muFR
//                double *muFL
//                double *muBR
//                double *muBL
// Return Type  : void
//
namespace Codegen {
void manipulability(double state[36], double *muFR, double *muFL, double *muBR,
                    double *muBL)
{
  double a__2[324];
  double a__4[324];
  double a__6[324];
  double a__8[324];
  double GeoJc_BL[108];
  double GeoJc_BR[108];
  double GeoJc_FL[108];
  double GeoJc_FR[108];
  double SIG_BL[108];
  double SIG_BR[108];
  double SIG_FL[108];
  double SIG_FR[108];
  double a__1[36];
  double a__3[36];
  double a__5[36];
  double a__7[36];
  double sigBL[6];
  double sigBR[6];
  double sigFL[6];
  double sigFR[6];
  (void)std::memset(&state[18], 0, 18U * (sizeof(double)));
  contactJacobians(state, GeoJc_FR, GeoJc_FL, GeoJc_BR, GeoJc_BL);
  //  Find SVD
  coder::b_svd(GeoJc_FR, a__1, SIG_FR, a__2);
  coder::b_svd(GeoJc_FL, a__3, SIG_FL, a__4);
  coder::b_svd(GeoJc_BR, a__5, SIG_BR, a__6);
  coder::b_svd(GeoJc_BL, a__7, SIG_BL, a__8);
  for (int ii{0}; ii < 6; ii++) {
    int sigFR_tmp;
    sigFR_tmp = ii + (6 * ii);
    sigFR[ii] = SIG_FR[sigFR_tmp];
    sigFL[ii] = SIG_FL[sigFR_tmp];
    sigBR[ii] = SIG_BR[sigFR_tmp];
    sigBL[ii] = SIG_BL[sigFR_tmp];
  }
  *muFR = coder::prod(sigFR);
  *muFL = coder::prod(sigFL);
  *muBR = coder::prod(sigBR);
  *muBL = coder::prod(sigBL);
}

} // namespace Codegen

//
// File trailer for manipulability.cpp
//
// [EOF]
//
