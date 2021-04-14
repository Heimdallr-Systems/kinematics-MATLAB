//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: power.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "power.h"
#include "floor.h"

// Function Declarations
namespace Codegen {
namespace coder {
static bool invalidPair(double a, double b);

static bool lt0(double a);

static bool notNaNOrInteger(double b);

} // namespace coder
} // namespace Codegen

// Function Definitions
//
// Arguments    : double a
//                double b
// Return Type  : bool
//
namespace Codegen {
namespace coder {
static bool invalidPair(double a, double b)
{
  bool p;
  if (lt0(a)) {
    if (notNaNOrInteger(b)) {
      p = true;
    } else {
      p = false;
    }
  } else {
    p = false;
  }
  return p;
}

//
// Arguments    : double a
// Return Type  : bool
//
static bool lt0(double a)
{
  return a < 0.0;
}

//
// Arguments    : double b
// Return Type  : bool
//
static bool notNaNOrInteger(double b)
{
  double d;
  d = b;
  internal::scalar::b_floor(&d);
  return d != b;
}

//
// Arguments    : double a
// Return Type  : bool
//
bool fltpower_domain_error(double a)
{
  return invalidPair(a, 2.0);
}

} // namespace coder
} // namespace Codegen

//
// File trailer for power.cpp
//
// [EOF]
//
