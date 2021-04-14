//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: CallTheDead.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

#ifndef CALLTHEDEAD_H
#define CALLTHEDEAD_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace Codegen {
extern void CallTheDead(const double Theta[12], double r_II_B_dead[3],
                        double T_I_B_dead[9], bool firstCall,
                        const bool b_legs_valid[4], bool prev_legs_valid[4],
                        double r_II_c_dead[12]);

}

#endif
//
// File trailer for CallTheDead.h
//
// [EOF]
//
