//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: centroid_codeGen.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "centroid_codeGen.h"
#include "circshift.h"
#include "sum.h"

// Function Definitions
//
// centroid_codeGen Calculate Centroid of a Polygon
//  Unlike the centroid() function, this is compatible
//  with MATLAB Coder
//
// Arguments    : const double x[4]
//                const double y[4]
//                double *Cx
//                double *Cy
// Return Type  : void
//
namespace Codegen {
void centroid_codeGen(const double x[4], const double y[4], double *Cx,
                      double *Cy)
{
  double b_commonBit_tmp[5];
  double b_x[5];
  double b_y[5];
  double commonBit[5];
  double commonBit_tmp[5];
  //  Finish the loop of the polygon
  b_x[4] = x[0];
  b_x[0] = x[0];
  b_y[0] = y[0];
  b_x[1] = x[1];
  b_y[1] = y[1];
  b_x[2] = x[2];
  b_y[2] = y[2];
  b_x[3] = x[3];
  b_y[3] = y[3];
  b_y[4] = y[0];
  //  Common portion that will be used multiple times.
  for (int i{0}; i < 5; i++) {
    commonBit_tmp[i] = b_x[i];
  }
  coder::circshift(commonBit_tmp);
  for (int i1{0}; i1 < 5; i1++) {
    b_commonBit_tmp[i1] = b_y[i1];
  }
  coder::circshift(b_commonBit_tmp);
  for (int i2{0}; i2 < 5; i2++) {
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    d = b_x[i2];
    d1 = b_commonBit_tmp[i2];
    d2 = commonBit_tmp[i2];
    d3 = b_y[i2];
    d4 = (d * d1) - (d2 * d3);
    commonBit[i2] = d4;
    d = (d + d2) * d4;
    b_x[i2] = d;
    d3 = (d3 + d1) * d4;
    b_y[i2] = d3;
  }
  double Cx_tmp;
  Cx_tmp = 1.0 / (6.0 * (0.5 * coder::sum(commonBit)));
  *Cx = Cx_tmp * coder::sum(b_x);
  *Cy = Cx_tmp * coder::sum(b_y);
}

} // namespace Codegen

//
// File trailer for centroid_codeGen.cpp
//
// [EOF]
//
