//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: contactJacobians.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef CONTACTJACOBIANS_H
#define CONTACTJACOBIANS_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
void contactJacobians(const double state[36], double GeoJc_FR[108],
                      double GeoJc_FL[108], double GeoJc_BR[108],
                      double GeoJc_BL[108]);

}

#endif
//
// File trailer for contactJacobians.h
//
// [EOF]
//
