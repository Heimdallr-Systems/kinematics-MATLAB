//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: recursiveKin.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef RECURSIVEKIN_H
#define RECURSIVEKIN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
void recursiveKin(const double dotgamma[18], const double ITn[9],
                  const double nTN[9], const double nnrN[3],
                  const double IN_hat[54], const double IN_tilde[54],
                  const double Jn[108], const double dotJn[108], double ITN[9],
                  double NNwI[3], double dotIIrN[3], double JN[108],
                  double dotJN[108]);

}

#endif
//
// File trailer for recursiveKin.h
//
// [EOF]
//
