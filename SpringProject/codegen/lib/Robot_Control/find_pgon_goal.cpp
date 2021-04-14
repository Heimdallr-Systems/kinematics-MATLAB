//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: find_pgon_goal.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "find_pgon_goal.h"
#include "cross.h"
#include "error.h"
#include "inpolygon.h"
#include "norm.h"
#include <stdio.h>

// Function Definitions
//
// Arguments    : const double r_II_c_FR[3]
//                const double r_II_c_FL[3]
//                const double r_II_c_BR[3]
//                const double r_II_c_BL[3]
//                double r_II_B[3]
//                unsigned char lifted_leg
//                double *x
//                double *y
// Return Type  : void
//
namespace Codegen {
void find_pgon_goal(const double r_II_c_FR[3], const double r_II_c_FL[3],
                    const double r_II_c_BR[3], const double r_II_c_BL[3],
                    double r_II_B[3], unsigned char lifted_leg, double *x,
                    double *y)
{
  static const double b_dv[3]{0.0, 0.0, 1.0};
  double b_r_II_B[3];
  double boundary_vec[3];
  double dir_to_pgon[3];
  double pgonx[3];
  double pgony[3];
  double d;
  double d1;
  unsigned short ii;
  bool exitg1;
  switch (lifted_leg) {
  case 1U:
    pgonx[0] = r_II_c_FL[0];
    pgonx[1] = r_II_c_BR[0];
    pgonx[2] = r_II_c_BL[0];
    pgony[0] = r_II_c_FL[1];
    pgony[1] = r_II_c_BR[1];
    pgony[2] = r_II_c_BL[1];
    boundary_vec[0] = r_II_c_FL[0] - r_II_c_BR[0];
    boundary_vec[1] = r_II_c_FL[1] - r_II_c_BR[1];
    boundary_vec[2] = r_II_c_FL[2] - r_II_c_BR[2];
    break;
  case 2U:
    pgonx[0] = r_II_c_FR[0];
    pgonx[1] = r_II_c_BR[0];
    pgonx[2] = r_II_c_BL[0];
    pgony[0] = r_II_c_FR[1];
    pgony[1] = r_II_c_BR[1];
    pgony[2] = r_II_c_BL[1];
    boundary_vec[0] = r_II_c_BL[0] - r_II_c_FR[0];
    boundary_vec[1] = r_II_c_BL[1] - r_II_c_FR[1];
    boundary_vec[2] = r_II_c_BL[2] - r_II_c_FR[2];
    break;
  case 3U:
    pgonx[0] = r_II_c_FR[0];
    pgonx[1] = r_II_c_FL[0];
    pgonx[2] = r_II_c_BL[0];
    pgony[0] = r_II_c_FR[1];
    pgony[1] = r_II_c_FL[1];
    pgony[2] = r_II_c_BL[1];
    boundary_vec[0] = r_II_c_FR[0] - r_II_c_BL[0];
    boundary_vec[1] = r_II_c_FR[1] - r_II_c_BL[1];
    boundary_vec[2] = r_II_c_FR[2] - r_II_c_BL[2];
    break;
  case 4U:
    pgonx[0] = r_II_c_FR[0];
    pgonx[1] = r_II_c_FL[0];
    pgonx[2] = r_II_c_BR[0];
    pgony[0] = r_II_c_FR[1];
    pgony[1] = r_II_c_FL[1];
    pgony[2] = r_II_c_BR[1];
    boundary_vec[0] = r_II_c_BR[0] - r_II_c_FL[0];
    boundary_vec[1] = r_II_c_BR[1] - r_II_c_FL[1];
    boundary_vec[2] = r_II_c_BR[2] - r_II_c_FL[2];
    break;
  default:
    (void)printf("Lifted_Leg is not set to a valid value\n");
    fflush(stdout);
    coder::error();
    break;
  }
  d = coder::c_norm(boundary_vec);
  boundary_vec[0] /= d;
  boundary_vec[1] /= d;
  boundary_vec[2] /= d;
  //  r_II_B_new = r_II_B;
  coder::cross(b_dv, boundary_vec, dir_to_pgon);
  b_r_II_B[0] = r_II_B[0] + dir_to_pgon[0];
  b_r_II_B[1] = r_II_B[1] + dir_to_pgon[1];
  b_r_II_B[2] = r_II_B[2] + dir_to_pgon[2];
  d1 = coder::c_norm(b_r_II_B);
  dir_to_pgon[0] /= d1;
  dir_to_pgon[1] /= d1;
  ii = 0U;
  exitg1 = false;
  while ((!exitg1) && ((static_cast<int>(ii)) <= 100)) {
    ii = static_cast<unsigned short>(
        static_cast<int>((static_cast<int>(ii)) + 1));
    if (coder::b_inpolygon(r_II_B[0], r_II_B[1], pgonx, pgony)) {
      r_II_B[0] += dir_to_pgon[0] * 0.035;
      r_II_B[1] += dir_to_pgon[1] * 0.035;
      exitg1 = true;
    } else {
      r_II_B[0] += dir_to_pgon[0] * 0.001;
      r_II_B[1] += dir_to_pgon[1] * 0.001;
    }
  }
  *x = r_II_B[0];
  *y = r_II_B[1];
}

} // namespace Codegen

//
// File trailer for find_pgon_goal.cpp
//
// [EOF]
//
