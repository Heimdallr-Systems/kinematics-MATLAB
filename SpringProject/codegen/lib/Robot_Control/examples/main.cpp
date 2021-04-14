//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "CallTheDead.h"
#include "Robot_Control.h"
#include "Robot_Control_initialize.h"
#include "Robot_Control_types.h"
#include "getUp.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Function Declarations
namespace Codegen {
static void d_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

}
static void argInit_12x1_real_T(double result[12]);

static void argInit_1x4_boolean_T(bool result[4]);

static void argInit_36x1_real_T(double result[36]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_3x3_real_T(double result[9]);

static void argInit_3x4_real_T(double result[12]);

static bool argInit_boolean_T();

static double argInit_real_T();

static unsigned char argInit_uint8_T();

static void main_CallTheDead();

static void main_Robot_Control();

static void main_getUp();

// Function Definitions
//
// Arguments    : const char *aFcnName
//                int aLineNum
// Return Type  : void
//
namespace Codegen {
static void d_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Example main does not support command line arguments.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

//
// Arguments    : double result[12]
// Return Type  : void
//
} // namespace Codegen
static void argInit_12x1_real_T(double result[12])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 12; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : bool result[4]
// Return Type  : void
//
static void argInit_1x4_boolean_T(bool result[4])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 4; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_boolean_T();
  }
}

//
// Arguments    : double result[36]
// Return Type  : void
//
static void argInit_36x1_real_T(double result[36])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 36; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 3; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (3 * idx1)] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[12]
// Return Type  : void
//
static void argInit_3x4_real_T(double result[12])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (3 * idx1)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : bool
//
static bool argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : unsigned char
//
static unsigned char argInit_uint8_T()
{
  return 0U;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_CallTheDead()
{
  double dv[12];
  double r_II_c_dead[12];
  double T_I_B_dead[9];
  double r_II_B_dead[3];
  bool b_legs_valid_tmp[4];
  bool legs_valid_tmp[4];
  // Initialize function 'CallTheDead' input arguments.
  // Initialize function input argument 'Theta'.
  // Initialize function input argument 'r_II_B_dead'.
  // Initialize function input argument 'T_I_B_dead'.
  // Initialize function input argument 'legs_valid'.
  argInit_1x4_boolean_T(legs_valid_tmp);
  // Initialize function input argument 'prev_legs_valid'.
  // Initialize function input argument 'r_II_c_dead'.
  // Call the entry-point 'CallTheDead'.
  argInit_3x1_real_T(r_II_B_dead);
  argInit_3x3_real_T(T_I_B_dead);
  argInit_3x4_real_T(r_II_c_dead);
  argInit_12x1_real_T(dv);
  b_legs_valid_tmp[0] = legs_valid_tmp[0];
  b_legs_valid_tmp[1] = legs_valid_tmp[1];
  b_legs_valid_tmp[2] = legs_valid_tmp[2];
  b_legs_valid_tmp[3] = legs_valid_tmp[3];
  Codegen::CallTheDead(dv, r_II_B_dead, T_I_B_dead, argInit_boolean_T(),
                       b_legs_valid_tmp, legs_valid_tmp, r_II_c_dead);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_Robot_Control()
{
  double dv2[36];
  double Theta1_d_out[4];
  double Theta2_d_out[4];
  double Theta3_d_out[4];
  double dv[3];
  double dv1[3];
  double r_II_B_d_temp_out[3];
  double phi_d_temp_out;
  bool bv[4];
  bool floor_toggle_out[4];
  bool legs_valid_out[4];
  // Initialize function 'Robot_Control' input arguments.
  // Initialize function input argument 'r_II_B_d'.
  // Initialize function input argument 'Euler_d'.
  // Initialize function input argument 'gamma_m'.
  // Initialize function input argument 'legs_on_gnd'.
  // Call the entry-point 'Robot_Control'.
  argInit_3x1_real_T(dv);
  argInit_3x1_real_T(dv1);
  argInit_36x1_real_T(dv2);
  argInit_1x4_boolean_T(bv);
  Codegen::Robot_Control(dv, dv1, dv2, argInit_boolean_T(), bv, Theta1_d_out,
                         Theta2_d_out, Theta3_d_out, &phi_d_temp_out,
                         r_II_B_d_temp_out, floor_toggle_out, legs_valid_out);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_getUp()
{
  double Theta_d[12];
  double dv[12];
  unsigned char stage;
  // Initialize function 'getUp' input arguments.
  // Initialize function input argument 'Theta'.
  // Call the entry-point 'getUp'.
  stage = argInit_uint8_T();
  argInit_12x1_real_T(dv);
  Codegen::getUp(dv, &stage, Theta_d);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int argc, char **)
{
  static Codegen::rtRunTimeErrorInfo
      emlrtRTEI{
          1,             // lineNo
          65,            // colNo
          "CallTheDead", // fName
          "D:\\Desktop\\Capstone\\kinematics-"
          "MATLAB\\SpringProject\\CallTheDead.m" // pName
      };
  if (argc > 1) {
    Codegen::d_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  // Initialize the application.
  // You do not need to do this more than one time.
  Codegen::Robot_Control_initialize();
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_CallTheDead();
  main_getUp();
  main_Robot_Control();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
