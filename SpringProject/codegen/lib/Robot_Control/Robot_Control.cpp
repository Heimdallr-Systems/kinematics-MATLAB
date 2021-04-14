//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Robot_Control.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "Robot_Control.h"
#include "Body_Pose_Controller.h"
#include "CPos_wrt_I.h"
#include "Leg_Controller.h"
#include "Leg_Controller_B.h"
#include "Robot_Control_rtwutil.h"
#include "Robot_Control_types.h"
#include "abs.h"
#include "all.h"
#include "centroid_codeGen.h"
#include "error.h"
#include "eye.h"
#include "find_pgon_goal.h"
#include "ifWhileCond.h"
#include "inpolygon.h"
#include "isequal.h"
#include "manipulability.h"
#include "norm.h"
#include "rotx.h"
#include "roty.h"
#include "rotz.h"
#include "sign.h"
#include "sort.h"
#include "step_planner_intelligent.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdio.h>

// Variable Definitions
namespace Codegen {
static bool waypoint_toggle;

static bool turn_toggle;

static unsigned char step_state;

static bool turn_state;

static unsigned char reached_centroid;

static unsigned char reached_rest_centroid;

static unsigned char step_needed;

static bool calc_manip;

static bool legs_valid[4];

static bool floor_toggle_not_empty;

static unsigned char legs_stepped;

static bool leg_reset_needed;

static unsigned char leg_index;

static bool endPoint_not_empty;

static double manip_vec[4];

static bool endPhi_not_empty;

static unsigned char is_turning;

static double phi_d_temp;

static double r_II_B_d_temp[3];

static double T_I_B_d_temp[9];

static double r_II_c_FR_0[3];

static double r_II_c_FL_0[3];

static double r_II_c_BR_0[3];

static double r_II_c_BL_0[3];

static double Theta1_d_midpt;

static double Theta2_d_midpt;

static double Theta3_d_midpt;

static double Theta1_d_reset;

static double Theta2_d_reset;

static double Theta3_d_reset;

static double r_II_c_current[3];

static double r_II_c_dstep[3];

static double Theta1_d[4];

static double Theta2_d[4];

static double Theta3_d[4];

} // namespace Codegen

// Function Declarations
namespace Codegen {
static unsigned char _u8_d_(double b);

static unsigned char _u8_u32_(unsigned int b);

} // namespace Codegen

// Function Definitions
//
// Arguments    : double b
// Return Type  : unsigned char
//
namespace Codegen {
static unsigned char _u8_d_(double b)
{
  unsigned char a;
  a = static_cast<unsigned char>(b);
  if ((b < 0.0) || ((static_cast<double>(a)) != std::floor(b))) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

//
// Arguments    : unsigned int b
// Return Type  : unsigned char
//
static unsigned char _u8_u32_(unsigned int b)
{
  unsigned char a;
  a = static_cast<unsigned char>(b);
  if ((static_cast<unsigned int>(a)) != b) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

//
// Controls Robot's walking algorithm
//    input: r_II_B_d, Euler_d, gamma_m
//    output: Theta_d (1-3), phi_d_temp & r_II_b_d_temp (orientation for
//    plotting), floor_toggle & legs_valid (for computing forces) NOTE: For the
//    real system, phi_d_temp & r_II_b_d_temp, floor_toggle & legs_valid can be
//    thrown away. For kinematic system, floor_toggle is not needed. For dynamic
//    system phi_d_temp % r_II_B_d_temp is not needed
//    r_II_B_d = [x_d;y_d;z_d];
//    Euler_d = [phi,theta,psi];
//    gamma_m = [Euler,r_II_B,Theta1,Theta2,Theta3];
//    Euler = [phi,theta,psi];
//    r_II_B = [x,y,z];
//    Theta1 = [Theta1FR,FL,BR,BL];
//    Theta2 = [Theta2FR,FL,BR,BL];
//    Theta3 = [Theta3FR,FL,BR,BL];
//
// Arguments    : const double r_II_B_d[3]
//                const double Euler_d[3]
//                const double gamma_m[36]
//                bool init_toggle
//                const bool legs_on_gnd[4]
//                double Theta1_d_out[4]
//                double Theta2_d_out[4]
//                double Theta3_d_out[4]
//                double *phi_d_temp_out
//                double r_II_B_d_temp_out[3]
//                bool floor_toggle_out[4]
//                bool legs_valid_out[4]
// Return Type  : void
//
void Robot_Control(const double r_II_B_d[3], const double Euler_d[3],
                   const double gamma_m[36], bool init_toggle,
                   const bool legs_on_gnd[4], double Theta1_d_out[4],
                   double Theta2_d_out[4], double Theta3_d_out[4],
                   double *phi_d_temp_out, double r_II_B_d_temp_out[3],
                   bool floor_toggle_out[4], bool legs_valid_out[4])
{
  static rtBoundsCheckInfo ab_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      462,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo b_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      533,             // lineNo
      110,             // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo bb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      462,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo c_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      533,             // lineNo
      128,             // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo cb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      438,             // lineNo
      92,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo d_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      728,             // lineNo
      26,              // colNo
      "Theta1_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo db_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      438,             // lineNo
      110,             // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo e_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      729,             // lineNo
      26,              // colNo
      "Theta2_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo eb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      438,             // lineNo
      128,             // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo emlrtBCI{
      1,               // iFirst
      4,               // iLast
      533,             // lineNo
      92,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo f_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      730,             // lineNo
      26,              // colNo
      "Theta3_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo fb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      431,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo g_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      734,             // lineNo
      26,              // colNo
      "Theta1_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo gb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      431,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo h_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      735,             // lineNo
      26,              // colNo
      "Theta2_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo hb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      431,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo i_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      736,             // lineNo
      26,              // colNo
      "Theta3_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo ib_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      676,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo j_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      732,             // lineNo
      27,              // colNo
      "Theta1_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo jb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      676,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo k_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      732,             // lineNo
      48,              // colNo
      "Theta2_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo kb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      676,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo l_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      732,             // lineNo
      69,              // colNo
      "Theta3_d",      // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      3    // checkKind
  };
  static rtBoundsCheckInfo lb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      646,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo m_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      526,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo mb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      646,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo n_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      526,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo nb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      646,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo o_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      526,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      616,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo p_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      501,             // lineNo
      92,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      616,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo q_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      501,             // lineNo
      110,             // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      616,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo r_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      501,             // lineNo
      128,             // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      586,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo s_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      494,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      586,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo t_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      494,             // lineNo
      61,              // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      586,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo u_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      494,             // lineNo
      79,              // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo v_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      469,             // lineNo
      92,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo w_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      469,             // lineNo
      110,             // colNo
      "Theta2",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo x_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      469,             // lineNo
      128,             // colNo
      "Theta3",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static rtBoundsCheckInfo y_emlrtBCI{
      1,               // iFirst
      4,               // iLast
      462,             // lineNo
      43,              // colNo
      "Theta1",        // aName
      "Robot_Control", // fName
      "D:\\Desktop\\Capstone\\kinematics-MATLAB\\SpringProject\\Robot_Control."
      "m", // pName
      0    // checkKind
  };
  static double endPoint[3];
  static double startPoint[3];
  static double endPhi;
  static double startPhi;
  static bool floor_toggle[4];
  double state[36];
  double r_II_c[12];
  double T_I_B[9];
  double T_I_B_d_tmp[9];
  double b_T_I_B_d_tmp[9];
  double b_dv[9];
  double b_dv1[9];
  double b_dv10[9];
  double b_dv2[9];
  double b_dv3[9];
  double b_dv7[9];
  double b_dv8[9];
  double b_dv9[9];
  double dv25[9];
  double Theta1[4];
  double Theta2[4];
  double Theta3[4];
  double a__1[4];
  double b_dv4[4];
  double b_dv5[4];
  double b_dv6[4];
  double b_r_II_c_FL[4];
  double b_r_II_c_FR[4];
  double c_r_II_c_FL[4];
  double c_r_II_c_FR[4];
  double dv11[4];
  double dv12[4];
  double dv13[4];
  double dv14[4];
  double dv15[4];
  double dv16[4];
  double dv17[4];
  double dv18[4];
  double dv19[4];
  double dv20[4];
  double dv21[4];
  double dv22[4];
  double dv26[4];
  double dv27[4];
  double dv28[4];
  double b_Theta1[3];
  double b_r_II_B[3];
  double b_r_II_B_d[3];
  double b_r_II_B_d_temp[3];
  double b_r_II_c_dstep[3];
  double c_r_II_B[3];
  double d_r_II_B[3];
  double dv23[3];
  double dv24[3];
  double dv29[3];
  double e_r_II_B[3];
  double f_r_II_B[3];
  double g_r_II_B[3];
  double h_r_II_B[3];
  double i_r_II_B[3];
  double j_r_II_B[3];
  double r_BB_c_reset_BL[3];
  double r_BB_c_reset_BR[3];
  double r_BB_c_reset_FL[3];
  double r_BB_c_reset_FR[3];
  double r_II_B[3];
  double r_II_c_BL[3];
  double r_II_c_BR[3];
  double r_II_c_FL[3];
  double r_II_c_FR[3];
  double b_Theta1_d_reset;
  double b_Theta2_d_reset;
  double b_Theta3_d_reset;
  double b_x;
  double b_y;
  double c_Theta1_d_reset;
  double c_Theta2_d_reset;
  double c_Theta3_d_reset;
  double c_x;
  double c_y;
  double d20;
  double d21;
  double d22;
  double d7;
  double d_Theta1_d_reset;
  double d_Theta2_d_reset;
  double d_Theta3_d_reset;
  double d_x;
  double d_y;
  double e_Theta1_d_reset;
  double e_Theta2_d_reset;
  double e_Theta3_d_reset;
  double e_x;
  double e_y;
  double f_Theta1_d_reset;
  double f_Theta2_d_reset;
  double f_Theta3_d_reset;
  double f_x;
  double f_y;
  double g_Theta1_d_reset;
  double g_Theta2_d_reset;
  double g_Theta3_d_reset;
  double g_x;
  double g_y;
  double h_Theta1_d_reset;
  double h_Theta2_d_reset;
  double h_Theta3_d_reset;
  double h_x;
  double h_y;
  double i_Theta1_d_reset;
  double i_Theta2_d_reset;
  double i_Theta3_d_reset;
  double i_x;
  double i_y;
  double muBL;
  double muBR;
  double muFL;
  double muFR;
  double x;
  double y;
  int iidx[4];
  bool bv[4];
  bool bv2[4];
  bool bv3[4];
  bool bv4[4];
  bool bv5[4];
  bool bv6[4];
  bool bv1[3];
  bool goal_inside_pgon;
  //  Variable Init
  //  leg step distance
  // %%CHANGE IF NEEDED%%%
  //  constants for mid-step resting position
  //  FR
  //  FL
  //  BR
  //  BL
  //  constants for hard stops
  //  FR
  //  Theta1_FR_Max = ;
  //  Theta1_FR_Min = ;
  //
  //  % FL
  //  Theta1_FL_Max = ;
  //  Theta1_FL_Min = ;
  //
  //  % BR
  //  Theta1_BR_Max = ;
  //  Theta1_BR_Min = ;
  //
  //  % BL
  //  Theta1_BL_Max = ;
  //  Theta1_BL_Min = ;
  //  initialize max bound for moving without steps
  //  persistent vars
  //  intialize toggle for determining direction of travel
  //  init toggle for determining direction of turning
  //  intialize toggle for determining which phase of step leg is in
  //  init toggle for determining which phase of turn robot is in
  //  initialize toggle for determining which state the body is in
  //  initialize toggle for determining which state the body is in when
  //  returning to a four-leg-defined polygon centroid initialize variable for
  //  determining which leg needs to be stepped next initialize toggle for
  //  determining if manipulability needs to be recalculated
  if (!floor_toggle_not_empty) {
    floor_toggle[0] = legs_valid[0];
    floor_toggle[1] = legs_valid[1];
    floor_toggle[2] = legs_valid[2];
    floor_toggle[3] = legs_valid[3];
    floor_toggle_not_empty = true;
  }
  //  Here begins the questionable initialization stuff. SHould be removed if
  //  possible leg_index=0 will fallthrough and cause function to not do
  //  anything
  //  TODO: Figure out how to remove these without breaking body pose controller
  //  Removing some vars causes errors in body pose controller
  //  Removing some vars causes a singular matrix
  //  Removing some vars casues legs to move too far forward
  if (init_toggle) {
    waypoint_toggle = false;
    //  intialize toggle for determining direction of travel
    turn_toggle = false;
    //  init toggle for determining direction of turning
    step_state = 0U;
    //  intialize toggle for determining which phase of step leg is in
    turn_state = false;
    //  init toggle for determining which phase of turn robot is in
    reached_centroid = 0U;
    //  initialize toggle for determining which state the body is in
    reached_rest_centroid = 1U;
    //  initialize toggle for determining which state the body is in when
    //  returning to a four-leg-defined polygon centroid
    step_needed = 1U;
    //  initialize variable for determining which leg needs to be stepped next
    calc_manip = true;
    //  initialize toggle for determining if manipulability needs to be
    //  recalculated
    legs_valid[0] = true;
    floor_toggle[0] = true;
    legs_valid[1] = true;
    floor_toggle[1] = true;
    legs_valid[2] = true;
    floor_toggle[2] = true;
    legs_valid[3] = true;
    floor_toggle[3] = true;
    legs_stepped = 0U;
    leg_reset_needed = false;
    coder::eye(T_I_B_d_temp);
    phi_d_temp = 0.0;
    is_turning = 0U;
  }
  //  Initialize state be the correct size
  (void)std::memset(&state[0], 0, 36U * (sizeof(double)));
  //  current
  r_II_B[0] = gamma_m[3];
  r_II_B[1] = gamma_m[4];
  r_II_B[2] = gamma_m[5];
  Theta1[0] = gamma_m[6];
  Theta1[1] = gamma_m[7];
  Theta1[2] = gamma_m[8];
  Theta1[3] = gamma_m[9];
  Theta2[0] = gamma_m[10];
  Theta2[1] = gamma_m[11];
  Theta2[2] = gamma_m[12];
  Theta2[3] = gamma_m[13];
  Theta3[0] = gamma_m[14];
  Theta3[1] = gamma_m[15];
  Theta3[2] = gamma_m[16];
  Theta3[3] = gamma_m[17];
  rotz(gamma_m[0], b_dv);
  roty(gamma_m[1], b_dv1);
  rotx(gamma_m[2], b_dv2);
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
  (void)std::copy(&gamma_m[0], &gamma_m[18], &state[0]);
  //  Constants for "resetting" legs
  //  FR
  r_BB_c_reset_FR[0] = 0.25;
  r_BB_c_reset_FR[1] = -0.25;
  r_BB_c_reset_FR[2] = -r_II_B_d[2];
  //  FL
  r_BB_c_reset_FL[0] = 0.25;
  r_BB_c_reset_FL[1] = 0.25;
  r_BB_c_reset_FL[2] = -r_II_B_d[2];
  //  BR
  r_BB_c_reset_BR[0] = -0.25;
  r_BB_c_reset_BR[1] = -0.25;
  r_BB_c_reset_BR[2] = -r_II_B_d[2];
  //  BL
  r_BB_c_reset_BL[0] = -0.25;
  r_BB_c_reset_BL[1] = 0.25;
  r_BB_c_reset_BL[2] = -r_II_B_d[2];
  //  desired
  roty(Euler_d[1], T_I_B_d_tmp);
  rotx(Euler_d[2], b_T_I_B_d_tmp);
  CPos_wrt_I(Theta1, Theta2, Theta3, T_I_B, r_II_B, r_II_c_FR, r_II_c_FL,
             r_II_c_BR, r_II_c_BL);
  r_II_c[0] = r_II_c_FR[0];
  r_II_c[3] = r_II_c_FL[0];
  r_II_c[6] = r_II_c_BR[0];
  r_II_c[9] = r_II_c_BL[0];
  r_II_c[1] = r_II_c_FR[1];
  r_II_c[4] = r_II_c_FL[1];
  r_II_c[7] = r_II_c_BR[1];
  r_II_c[10] = r_II_c_BL[1];
  r_II_c[2] = r_II_c_FR[2];
  r_II_c[5] = r_II_c_FL[2];
  r_II_c[8] = r_II_c_BR[2];
  r_II_c[11] = r_II_c_BL[2];
  //  Check if step is needed
  b_r_II_c_FL[0] = r_II_c_FL[0];
  b_r_II_c_FL[1] = r_II_c_FR[0];
  b_r_II_c_FL[2] = r_II_c_BR[0];
  b_r_II_c_FL[3] = r_II_c_BL[0];
  c_r_II_c_FL[0] = r_II_c_FL[1];
  c_r_II_c_FL[1] = r_II_c_FR[1];
  c_r_II_c_FL[2] = r_II_c_BR[1];
  c_r_II_c_FL[3] = r_II_c_BL[1];
  goal_inside_pgon =
      coder::inpolygon(gamma_m[3], gamma_m[4], b_r_II_c_FL, c_r_II_c_FL);
  //  manipulability calculations
  //  find the least manipulable leg, start stepping that leg. then
  //  step the next least manipulable leg until the last leg is stepped
  // TODO: COnvert to seperate function
  //  TODO: Why not just run this every time?
  if (((static_cast<int>(step_state)) == 0) &&
      ((static_cast<int>(reached_rest_centroid)) == 1)) {
    if (calc_manip) {
      manipulability(state, &muFR, &muFL, &muBR, &muBL);
      a__1[0] = muFR;
      a__1[1] = muFL;
      a__1[2] = muBR;
      a__1[3] = muBL;
      coder::internal::sort(a__1, iidx);
      manip_vec[0] = static_cast<double>(iidx[0]);
      manip_vec[1] = static_cast<double>(iidx[1]);
      manip_vec[2] = static_cast<double>(iidx[2]);
      manip_vec[3] = static_cast<double>(iidx[3]);
      calc_manip = false;
    }
    switch (step_needed) {
    case 1U:
      leg_index = _u8_d_(std::round(manip_vec[0]));
      step_needed = 2U;
      break;
    case 2U:
      leg_index = _u8_d_(std::round(manip_vec[1]));
      step_needed = 3U;
      break;
    case 3U:
      leg_index = _u8_d_(std::round(manip_vec[2]));
      step_needed = 4U;
      break;
    case 4U:
      leg_index = _u8_d_(std::round(manip_vec[3]));
      step_needed = 1U;
      calc_manip = true;
      break;
    default:
      (void)printf("Step_needed is set to an incorrect value");
      fflush(stdout);
      coder::error();
      break;
    }
  }
  //  waypoint section
  if ((!waypoint_toggle) || (!endPoint_not_empty)) {
    startPoint[0] = gamma_m[3];
    endPoint[0] = r_II_B_d[0];
    startPoint[1] = gamma_m[4];
    endPoint[1] = r_II_B_d[1];
    startPoint[2] = gamma_m[5];
    endPoint[2] = r_II_B_d[2];
    endPoint_not_empty = true;
    waypoint_toggle = true;
  } else {
    waypoint_toggle =
        ((coder::b_isequal(endPoint, r_II_B_d)) && waypoint_toggle);
  }
  if ((!turn_toggle) || (!endPhi_not_empty)) {
    startPhi = gamma_m[0];
    endPhi = Euler_d[0];
    endPhi_not_empty = true;
    turn_toggle = true;
  } else {
    turn_toggle = ((endPhi == Euler_d[0]) && turn_toggle);
  }
  //  Turn Needed Algorithm
  if (coder::b_abs(endPhi - startPhi) > 0.20943951023931953) {
    double d6;
    d6 = coder::b_abs(Euler_d[0] - gamma_m[0]);
    if (d6 < 0.31415926535897931) {
      if ((!turn_state) && (!leg_reset_needed)) {
        is_turning = 1U;
        phi_d_temp = Euler_d[0];
        leg_reset_needed = false;
        turn_state = true;
      } else if (turn_state && (d6 < 0.05)) {
        turn_state = false;
        startPhi = gamma_m[0];
        leg_reset_needed = true;
        //  reset legs
        is_turning = 2U;
      } else {
        /* no actions */
      }

      //  TODO: Replace turn dir calc with if statement to reduce number of
      //  variables that are of type double
    } else if ((!turn_state) && (!leg_reset_needed)) {
      is_turning = 1U;
      d7 = Euler_d[0] - gamma_m[0];
      coder::b_sign(&d7);
      phi_d_temp = gamma_m[0] + ((d7 * 3.1415926535897931) / 15.0);
      rotz(phi_d_temp, b_dv);
      for (int i3{0}; i3 < 3; i3++) {
        double d10;
        double d14;
        double d15;
        double d16;
        double d8;
        double d9;
        d8 = b_dv[i3];
        d9 = b_dv[i3 + 3];
        d10 = b_dv[i3 + 6];
        for (int i4{0}; i4 < 3; i4++) {
          b_dv1[i3 + (3 * i4)] =
              ((d8 * T_I_B_d_tmp[3 * i4]) + (d9 * T_I_B_d_tmp[(3 * i4) + 1])) +
              (d10 * T_I_B_d_tmp[(3 * i4) + 2]);
        }
        d14 = b_dv1[i3];
        d15 = b_dv1[i3 + 3];
        d16 = b_dv1[i3 + 6];
        for (int i6{0}; i6 < 3; i6++) {
          T_I_B_d_temp[i3 + (3 * i6)] = ((d14 * b_T_I_B_d_tmp[3 * i6]) +
                                         (d15 * b_T_I_B_d_tmp[(3 * i6) + 1])) +
                                        (d16 * b_T_I_B_d_tmp[(3 * i6) + 2]);
        }
      }
      turn_state = true;
    } else if (turn_state) {
      is_turning = 1U;
      if (coder::b_abs(phi_d_temp - gamma_m[0]) < 0.05) {
        turn_state = false;
        startPhi = gamma_m[0];
        leg_reset_needed = true;
        //  reset legs
      }
    } else {
      /* no actions */
    }
    //     %% Turn Stepping Section
    if (leg_reset_needed) {
      bool guard1{false};
      bool guard2{false};
      bool guard3{false};
      bool guard4{false};
      //  step to reset legs
      guard1 = false;
      guard2 = false;
      guard3 = false;
      guard4 = false;
      switch (leg_index) {
      case 1:
        legs_valid[0] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          c_r_II_B[0] = r_II_B[0];
          c_r_II_B[1] = r_II_B[1];
          c_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, c_r_II_B,
                         leg_index, &x, &y);
          r_II_B_d_temp[0] = x;
          r_II_B_d_temp[1] = y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = 0.78539816339744828;
          Theta2_d_midpt = -0.78539816339744828;
          Theta3_d_midpt = 2.3561944901923448;
          step_state = 1U;
          r_II_c_FR_0[0] = r_II_c_FR[0];
          r_II_c_FR_0[1] = r_II_c_FR[1];
          r_II_c_FR_0[2] = r_II_c_FR[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &fb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &gb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &hb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 3U;
            Leg_Controller_B(r_BB_c_reset_FR, static_cast<unsigned char>(1U),
                             &f_Theta1_d_reset, &f_Theta2_d_reset,
                             &f_Theta3_d_reset);
            Theta1_d_reset = f_Theta1_d_reset;
            Theta2_d_reset = f_Theta2_d_reset;
            Theta3_d_reset = f_Theta3_d_reset;
          }
          break;
        case 3:
          //  stepping towards goal now
          Leg_Controller_B(r_BB_c_reset_FR, static_cast<unsigned char>(1U),
                           &b_Theta1_d_reset, &b_Theta2_d_reset,
                           &b_Theta3_d_reset);
          Theta1_d_reset = b_Theta1_d_reset;
          Theta2_d_reset = b_Theta2_d_reset;
          Theta3_d_reset = b_Theta3_d_reset;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &cb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &db_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &eb_emlrtBCI);
          }
          if ((r_II_c_FR[2] <= 0.0) || (legs_on_gnd[0])) {
            guard1 = true;
          } else {
            dv24[0] =
                Theta1_d_reset - Theta1[(static_cast<int>(leg_index)) - 1];
            dv24[1] =
                Theta2_d_reset - Theta2[(static_cast<int>(leg_index)) - 1];
            dv24[2] =
                Theta3_d_reset - Theta3[(static_cast<int>(leg_index)) - 1];
            if (coder::c_norm(dv24) < 0.005) {
              guard1 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 2:
        legs_valid[1] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          d_r_II_B[0] = r_II_B[0];
          d_r_II_B[1] = r_II_B[1];
          d_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, d_r_II_B,
                         leg_index, &b_x, &b_y);
          r_II_B_d_temp[0] = b_x;
          r_II_B_d_temp[1] = b_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = -0.78539816339744828;
          Theta2_d_midpt = 0.78539816339744828;
          Theta3_d_midpt = -2.3561944901923448;
          step_state = 1U;
          r_II_c_FL_0[0] = r_II_c_FL[0];
          r_II_c_FL_0[1] = r_II_c_FL[1];
          r_II_c_FL_0[2] = r_II_c_FL[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &y_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &ab_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &bb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 3U;
            Leg_Controller_B(r_BB_c_reset_FL, static_cast<unsigned char>(2U),
                             &g_Theta1_d_reset, &g_Theta2_d_reset,
                             &g_Theta3_d_reset);
            Theta1_d_reset = g_Theta1_d_reset;
            Theta2_d_reset = g_Theta2_d_reset;
            Theta3_d_reset = g_Theta3_d_reset;
          }
          break;
        case 3:
          //  stepping towards goal now
          Leg_Controller_B(r_BB_c_reset_FL, static_cast<unsigned char>(2U),
                           &c_Theta1_d_reset, &c_Theta2_d_reset,
                           &c_Theta3_d_reset);
          Theta1_d_reset = c_Theta1_d_reset;
          Theta2_d_reset = c_Theta2_d_reset;
          Theta3_d_reset = c_Theta3_d_reset;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &v_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &w_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &x_emlrtBCI);
          }
          if ((r_II_c_FL[2] <= 0.0) || (legs_on_gnd[1])) {
            guard2 = true;
          } else {
            dv24[0] =
                Theta1_d_reset - Theta1[(static_cast<int>(leg_index)) - 1];
            dv24[1] =
                Theta2_d_reset - Theta2[(static_cast<int>(leg_index)) - 1];
            dv24[2] =
                Theta3_d_reset - Theta3[(static_cast<int>(leg_index)) - 1];
            if (coder::c_norm(dv24) < 0.005) {
              guard2 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 3:
        legs_valid[2] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          e_r_II_B[0] = r_II_B[0];
          e_r_II_B[1] = r_II_B[1];
          e_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, e_r_II_B,
                         leg_index, &c_x, &c_y);
          r_II_B_d_temp[0] = c_x;
          r_II_B_d_temp[1] = c_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = -0.78539816339744828;
          Theta2_d_midpt = -0.78539816339744828;
          Theta3_d_midpt = 2.3561944901923448;
          step_state = 1U;
          r_II_c_BR_0[0] = r_II_c_BR[0];
          r_II_c_BR_0[1] = r_II_c_BR[1];
          r_II_c_BR_0[2] = r_II_c_BR[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &s_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &t_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &u_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 3U;
            Leg_Controller_B(r_BB_c_reset_BR, static_cast<unsigned char>(3U),
                             &h_Theta1_d_reset, &h_Theta2_d_reset,
                             &h_Theta3_d_reset);
            Theta1_d_reset = h_Theta1_d_reset;
            Theta2_d_reset = h_Theta2_d_reset;
            Theta3_d_reset = h_Theta3_d_reset;
          }
          break;
        case 3:
          //  stepping towards goal now
          Leg_Controller_B(r_BB_c_reset_BR, static_cast<unsigned char>(3U),
                           &d_Theta1_d_reset, &d_Theta2_d_reset,
                           &d_Theta3_d_reset);
          Theta1_d_reset = d_Theta1_d_reset;
          Theta2_d_reset = d_Theta2_d_reset;
          Theta3_d_reset = d_Theta3_d_reset;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &p_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &q_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &r_emlrtBCI);
          }
          if ((r_II_c_BR[2] <= 0.0) || (legs_on_gnd[2])) {
            guard3 = true;
          } else {
            dv24[0] =
                Theta1_d_reset - Theta1[(static_cast<int>(leg_index)) - 1];
            dv24[1] =
                Theta2_d_reset - Theta2[(static_cast<int>(leg_index)) - 1];
            dv24[2] =
                Theta3_d_reset - Theta3[(static_cast<int>(leg_index)) - 1];
            if (coder::c_norm(dv24) < 0.005) {
              guard3 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 4:
        legs_valid[3] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          f_r_II_B[0] = r_II_B[0];
          f_r_II_B[1] = r_II_B[1];
          f_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, f_r_II_B,
                         leg_index, &d_x, &d_y);
          r_II_B_d_temp[0] = d_x;
          r_II_B_d_temp[1] = d_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = 0.78539816339744828;
          Theta2_d_midpt = 0.78539816339744828;
          Theta3_d_midpt = -2.3561944901923448;
          step_state = 1U;
          r_II_c_BL_0[0] = r_II_c_BL[0];
          r_II_c_BL_0[1] = r_II_c_BL[1];
          r_II_c_BL_0[2] = r_II_c_BL[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &m_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &n_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &o_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 3U;
            Leg_Controller_B(r_BB_c_reset_BL, static_cast<unsigned char>(4U),
                             &i_Theta1_d_reset, &i_Theta2_d_reset,
                             &i_Theta3_d_reset);
            Theta1_d_reset = i_Theta1_d_reset;
            Theta2_d_reset = i_Theta2_d_reset;
            Theta3_d_reset = i_Theta3_d_reset;
          }
          break;
        case 3:
          //  stepping towards goal now
          Leg_Controller_B(r_BB_c_reset_BL, static_cast<unsigned char>(4U),
                           &e_Theta1_d_reset, &e_Theta2_d_reset,
                           &e_Theta3_d_reset);
          Theta1_d_reset = e_Theta1_d_reset;
          Theta2_d_reset = e_Theta2_d_reset;
          Theta3_d_reset = e_Theta3_d_reset;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4, &emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &b_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &c_emlrtBCI);
          }
          if ((r_II_c_BL[2] <= 0.0) || (legs_on_gnd[3])) {
            guard4 = true;
          } else {
            dv24[0] =
                Theta1_d_reset - Theta1[(static_cast<int>(leg_index)) - 1];
            dv24[1] =
                Theta2_d_reset - Theta2[(static_cast<int>(leg_index)) - 1];
            dv24[2] =
                Theta3_d_reset - Theta3[(static_cast<int>(leg_index)) - 1];
            if (coder::c_norm(dv24) < 0.005) {
              guard4 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      default:
        /* no actions */
        break;
      }
      if (guard4) {
        step_state = 0U;
        legs_valid[3] = true;
        leg_index = 0U;
        floor_toggle[3] = true;
        reached_centroid = 0U;
        reached_rest_centroid = 0U;
        // if error reached
        legs_stepped = _u8_u32_((static_cast<unsigned int>(legs_stepped)) + 1U);
      }
      if (guard3) {
        legs_valid[2] = true;
        step_state = 0U;
        leg_index = 0U;
        floor_toggle[2] = true;
        reached_centroid = 0U;
        reached_rest_centroid = 0U;
        // if error reached
        legs_stepped = _u8_u32_((static_cast<unsigned int>(legs_stepped)) + 1U);
      }
      if (guard2) {
        step_state = 0U;
        legs_valid[1] = true;
        leg_index = 0U;
        floor_toggle[1] = true;
        reached_centroid = 0U;
        reached_rest_centroid = 0U;
        // if error reached
        legs_stepped = _u8_u32_((static_cast<unsigned int>(legs_stepped)) + 1U);
      }
      if (guard1) {
        step_state = 0U;
        legs_valid[0] = true;
        leg_index = 0U;
        floor_toggle[0] = true;
        reached_centroid = 0U;
        reached_rest_centroid = 0U;
        legs_stepped = _u8_u32_((static_cast<unsigned int>(legs_stepped)) + 1U);
      }
      if ((static_cast<int>(legs_stepped)) == 4) {
        legs_stepped = 0U;
        leg_reset_needed = false;
        if ((static_cast<int>(is_turning)) == 2) {
          is_turning = 0U;
        }
      }
    }
    //     %% walking forward section
  } else {
    bool guard1;
    bool guard2;
    bool guard3;
    bool guard4;
    bool guard5{false};
    b_r_II_B_d[0] = r_II_B_d[0] - gamma_m[3];
    b_r_II_B_d[1] = r_II_B_d[1] - gamma_m[4];
    b_r_II_B_d[2] = r_II_B_d[2] - gamma_m[5];
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    guard5 = false;
    if (coder::c_norm(b_r_II_B_d) >= 0.06) {
      guard5 = true;
    } else if (((static_cast<int>(step_state)) != 0) || (!goal_inside_pgon)) {
      guard5 = true;
    } else {
      //  move with normal body pose controller
      rotz(Euler_d[0], b_dv);
      for (int b_i{0}; b_i < 3; b_i++) {
        double d11;
        double d12;
        double d13;
        double d17;
        double d18;
        double d19;
        b_r_II_B_d_temp[b_i] = r_II_B_d[b_i];
        d11 = b_dv[b_i];
        d12 = b_dv[b_i + 3];
        d13 = b_dv[b_i + 6];
        for (int i5{0}; i5 < 3; i5++) {
          b_dv1[b_i + (3 * i5)] = ((d11 * T_I_B_d_tmp[3 * i5]) +
                                   (d12 * T_I_B_d_tmp[(3 * i5) + 1])) +
                                  (d13 * T_I_B_d_tmp[(3 * i5) + 2]);
        }
        d17 = b_dv1[b_i];
        d18 = b_dv1[b_i + 3];
        d19 = b_dv1[b_i + 6];
        for (int i7{0}; i7 < 3; i7++) {
          b_dv[b_i + (3 * i7)] = ((d17 * b_T_I_B_d_tmp[3 * i7]) +
                                  (d18 * b_T_I_B_d_tmp[(3 * i7) + 1])) +
                                 (d19 * b_T_I_B_d_tmp[(3 * i7) + 2]);
        }
      }
      bv[0] = floor_toggle[0];
      bv[1] = floor_toggle[1];
      bv[2] = floor_toggle[2];
      bv[3] = floor_toggle[3];
      Body_Pose_Controller(r_II_c, b_dv, b_r_II_B_d_temp, r_II_B, bv, b_dv4,
                           b_dv5, b_dv6);
      Theta3_d[0] = b_dv6[0];
      Theta3_d[1] = b_dv6[1];
      Theta3_d[2] = b_dv6[2];
      Theta3_d[3] = b_dv6[3];
      Theta2_d[0] = b_dv5[0];
      Theta2_d[1] = b_dv5[1];
      Theta2_d[2] = b_dv5[2];
      Theta2_d[3] = b_dv5[3];
      Theta1_d[0] = b_dv4[0];
      Theta1_d[1] = b_dv4[1];
      Theta1_d[2] = b_dv4[2];
      Theta1_d[3] = b_dv4[3];
      r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
      r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
      r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
    }
    if (guard5) {
      //  determine direction of travel
      //  the direction of travel is computed at the very beginning or when
      //  the desired body position changes
      if (!waypoint_toggle) {
        startPoint[0] = gamma_m[3];
        endPoint[0] = r_II_B_d[0];
        startPoint[1] = gamma_m[4];
        endPoint[1] = r_II_B_d[1];
        startPoint[2] = gamma_m[5];
        endPoint[2] = r_II_B_d[2];
        waypoint_toggle = true;
      } else {
        bv1[0] = (endPoint[0] != r_II_B_d[0]);
        bv1[1] = (endPoint[1] != r_II_B_d[1]);
        bv1[2] = (endPoint[2] != r_II_B_d[2]);
        waypoint_toggle =
            ((!coder::internal::ifWhileCond(bv1)) && waypoint_toggle);
      }
      //         %% leg stepping algorithm
      //  FR leg needs to step
      switch (leg_index) {
      case 1:
        legs_valid[0] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          g_r_II_B[0] = r_II_B[0];
          g_r_II_B[1] = r_II_B[1];
          g_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, g_r_II_B,
                         leg_index, &f_x, &f_y);
          r_II_B_d_temp[0] = f_x;
          r_II_B_d_temp[1] = f_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = 0.78539816339744828;
          Theta2_d_midpt = -0.78539816339744828;
          Theta3_d_midpt = 2.3561944901923448;
          step_state = 1U;
          r_II_c_FR_0[0] = r_II_c_FR[0];
          r_II_c_FR_0[1] = r_II_c_FR[1];
          r_II_c_FR_0[2] = r_II_c_FR[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &rb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &sb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &tb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 2U;
            step_planner_intelligent(startPoint, endPoint, r_II_c_FR_0, 0.12,
                                     r_II_c_dstep);
            r_II_c_current[0] = r_II_c_FR_0[0];
            r_II_c_current[1] = r_II_c_FR_0[1];
            r_II_c_current[2] = r_II_c_FR_0[2];
          }
          break;
        case 2:
          //  stepping towards goal now
          if ((r_II_c_FR[2] <= 0.0) || (legs_on_gnd[0])) {
            guard1 = true;
          } else {
            dv23[0] = r_II_c_dstep[0] - r_II_c_FR[0];
            dv23[1] = r_II_c_dstep[1] - r_II_c_FR[1];
            dv23[2] = r_II_c_dstep[2] - r_II_c_FR[2];
            if (coder::c_norm(dv23) < 0.005) {
              guard1 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 2:
        legs_valid[1] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          h_r_II_B[0] = r_II_B[0];
          h_r_II_B[1] = r_II_B[1];
          h_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, h_r_II_B,
                         leg_index, &g_x, &g_y);
          r_II_B_d_temp[0] = g_x;
          r_II_B_d_temp[1] = g_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = -0.78539816339744828;
          Theta2_d_midpt = 0.78539816339744828;
          Theta3_d_midpt = -2.3561944901923448;
          step_state = 1U;
          r_II_c_FL_0[0] = r_II_c_FL[0];
          r_II_c_FL_0[1] = r_II_c_FL[1];
          r_II_c_FL_0[2] = r_II_c_FL[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &ob_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &pb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &qb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 2U;
            step_planner_intelligent(startPoint, endPoint, r_II_c_FL_0, 0.12,
                                     r_II_c_dstep);
            r_II_c_current[0] = r_II_c_FL_0[0];
            r_II_c_current[1] = r_II_c_FL_0[1];
            r_II_c_current[2] = r_II_c_FL_0[2];
          }
          break;
        case 2:
          //  stepping towards goal now
          if ((r_II_c_FL[2] <= 0.0) || (legs_on_gnd[1])) {
            guard2 = true;
          } else {
            dv23[0] = r_II_c_dstep[0] - r_II_c_FL[0];
            dv23[1] = r_II_c_dstep[1] - r_II_c_FL[1];
            dv23[2] = r_II_c_dstep[2] - r_II_c_FL[2];
            if (coder::c_norm(dv23) < 0.005) {
              guard2 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 3:
        legs_valid[2] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          i_r_II_B[0] = r_II_B[0];
          i_r_II_B[1] = r_II_B[1];
          i_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, i_r_II_B,
                         leg_index, &h_x, &h_y);
          r_II_B_d_temp[0] = h_x;
          r_II_B_d_temp[1] = h_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = -0.78539816339744828;
          Theta2_d_midpt = -0.78539816339744828;
          Theta3_d_midpt = 2.3561944901923448;
          step_state = 1U;
          r_II_c_BR_0[0] = r_II_c_BR[0];
          r_II_c_BR_0[1] = r_II_c_BR[1];
          r_II_c_BR_0[2] = r_II_c_BR[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &lb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &mb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &nb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 2U;
            step_planner_intelligent(startPoint, endPoint, r_II_c_BR_0, 0.12,
                                     r_II_c_dstep);
            r_II_c_current[0] = r_II_c_BR_0[0];
            r_II_c_current[1] = r_II_c_BR_0[1];
            r_II_c_current[2] = r_II_c_BR_0[2];
          }
          break;
        case 2:
          //  stepping towards goal now
          if ((r_II_c_BR[2] <= 0.0) || (legs_on_gnd[2])) {
            guard3 = true;
          } else {
            dv23[0] = r_II_c_dstep[0] - r_II_c_BR[0];
            dv23[1] = r_II_c_dstep[1] - r_II_c_BR[1];
            dv23[2] = r_II_c_dstep[2] - r_II_c_BR[2];
            if (coder::c_norm(dv23) < 0.005) {
              guard3 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      case 4:
        legs_valid[3] = false;
        if ((static_cast<int>(reached_centroid)) == 0) {
          j_r_II_B[0] = r_II_B[0];
          j_r_II_B[1] = r_II_B[1];
          j_r_II_B[2] = r_II_B[2];
          find_pgon_goal(r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL, j_r_II_B,
                         leg_index, &i_x, &i_y);
          r_II_B_d_temp[0] = i_x;
          r_II_B_d_temp[1] = i_y;
          r_II_B_d_temp[2] = r_II_B_d[2];
        }
        switch (step_state) {
        case 0:
          //  hasn't started stepping yet
          Theta1_d_midpt = 0.78539816339744828;
          Theta2_d_midpt = 0.78539816339744828;
          Theta3_d_midpt = -2.3561944901923448;
          step_state = 1U;
          r_II_c_BL_0[0] = r_II_c_BL[0];
          r_II_c_BL_0[1] = r_II_c_BL[1];
          r_II_c_BL_0[2] = r_II_c_BL[2];
          break;
        case 1:
          //  moving towards midpoint
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &ib_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &jb_emlrtBCI);
          }
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &kb_emlrtBCI);
          }
          b_Theta1[0] = Theta1[(static_cast<int>(leg_index)) - 1];
          b_Theta1[1] = Theta2[(static_cast<int>(leg_index)) - 1];
          b_Theta1[2] = Theta3[(static_cast<int>(leg_index)) - 1];
          b_Theta1[0] -= Theta1_d_midpt;
          b_Theta1[1] -= Theta2_d_midpt;
          b_Theta1[2] -= Theta3_d_midpt;
          if (coder::c_norm(b_Theta1) < 0.2) {
            //  reached midpoint
            step_state = 2U;
            step_planner_intelligent(startPoint, endPoint, r_II_c_BL_0, 0.12,
                                     r_II_c_dstep);
            r_II_c_current[0] = r_II_c_BL_0[0];
            r_II_c_current[1] = r_II_c_BL_0[1];
            r_II_c_current[2] = r_II_c_BL_0[2];
          }
          break;
        case 2:
          //  stepping towards goal now
          if ((r_II_c_BL[2] <= 0.0) || (legs_on_gnd[3])) {
            guard4 = true;
          } else {
            dv23[0] = r_II_c_dstep[0] - r_II_c_BL[0];
            dv23[1] = r_II_c_dstep[1] - r_II_c_BL[1];
            dv23[2] = r_II_c_dstep[2] - r_II_c_BL[2];
            if (coder::c_norm(dv23) < 0.005) {
              guard4 = true;
            }
          }
          break;
        default:
          /* no actions */
          break;
        }
        break;
      default:
        /* no actions */
        break;
      }
    }
    if (guard4) {
      step_state = 0U;
      legs_valid[3] = true;
      leg_index = 0U;
      floor_toggle[3] = true;
      reached_centroid = 0U;
      reached_rest_centroid = 0U;
    }
    if (guard3) {
      legs_valid[2] = true;
      step_state = 0U;
      leg_index = 0U;
      floor_toggle[2] = true;
      reached_centroid = 0U;
      reached_rest_centroid = 0U;
    }
    if (guard2) {
      step_state = 0U;
      legs_valid[1] = true;
      leg_index = 0U;
      floor_toggle[1] = true;
      reached_centroid = 0U;
      reached_rest_centroid = 0U;
    }
    if (guard1) {
      step_state = 0U;
      legs_valid[0] = true;
      leg_index = 0U;
      floor_toggle[0] = true;
      reached_centroid = 0U;
      reached_rest_centroid = 0U;
    }
  }
  //  centroid moving/balance section, and commanding step (assigning thetad's)
  switch (reached_rest_centroid) {
  case 0:
    //  needs to move back to resting 4-legged position to find new leg to move
    b_r_II_c_FR[0] = r_II_c_FR[0];
    b_r_II_c_FR[1] = r_II_c_FL[0];
    b_r_II_c_FR[2] = r_II_c_BL[0];
    b_r_II_c_FR[3] = r_II_c_BR[0];
    c_r_II_c_FR[0] = r_II_c_FR[1];
    c_r_II_c_FR[1] = r_II_c_FL[1];
    c_r_II_c_FR[2] = r_II_c_BL[1];
    c_r_II_c_FR[3] = r_II_c_BR[1];
    centroid_codeGen(b_r_II_c_FR, c_r_II_c_FR, &e_x, &e_y);
    r_II_B_d_temp[0] = e_x;
    r_II_B_d_temp[1] = e_y;
    r_II_B_d_temp[2] = r_II_B_d[2];
    b_r_II_B_d_temp[0] = r_II_B_d_temp[0];
    b_r_II_B_d_temp[1] = r_II_B_d_temp[1];
    b_r_II_B_d_temp[2] = r_II_B_d_temp[2];
    bv6[0] = true;
    bv6[1] = true;
    bv6[2] = true;
    bv6[3] = true;
    (void)std::copy(&T_I_B_d_temp[0], &T_I_B_d_temp[9], &dv25[0]);
    Body_Pose_Controller(r_II_c, dv25, b_r_II_B_d_temp, r_II_B, bv6, dv26, dv27,
                         dv28);
    Theta3_d[0] = dv28[0];
    Theta3_d[1] = dv28[1];
    Theta3_d[2] = dv28[2];
    Theta3_d[3] = dv28[3];
    Theta2_d[0] = dv27[0];
    Theta2_d[1] = dv27[1];
    Theta2_d[2] = dv27[2];
    Theta2_d[3] = dv27[3];
    Theta1_d[0] = dv26[0];
    Theta1_d[1] = dv26[1];
    Theta1_d[2] = dv26[2];
    Theta1_d[3] = dv26[3];
    r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
    r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
    r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
    reached_rest_centroid = 2U;
    break;
  case 2:
    //  moving towards resting, or inbetween-step body pose
    b_r_II_B[0] = gamma_m[3] - r_II_B_d_temp[0];
    b_r_II_B_d_temp[0] = r_II_B_d_temp[0];
    b_r_II_B[1] = gamma_m[4] - r_II_B_d_temp[1];
    b_r_II_B_d_temp[1] = r_II_B_d_temp[1];
    b_r_II_B[2] = gamma_m[5] - r_II_B_d_temp[2];
    b_r_II_B_d_temp[2] = r_II_B_d_temp[2];
    (void)std::copy(&T_I_B_d_temp[0], &T_I_B_d_temp[9], &b_dv7[0]);
    bv2[0] = floor_toggle[0];
    bv2[1] = floor_toggle[1];
    bv2[2] = floor_toggle[2];
    bv2[3] = floor_toggle[3];
    Body_Pose_Controller(r_II_c, b_dv7, b_r_II_B_d_temp, r_II_B, bv2, dv11,
                         dv12, dv13);
    Theta3_d[0] = dv13[0];
    Theta3_d[1] = dv13[1];
    Theta3_d[2] = dv13[2];
    Theta3_d[3] = dv13[3];
    Theta2_d[0] = dv12[0];
    Theta2_d[1] = dv12[1];
    Theta2_d[2] = dv12[2];
    Theta2_d[3] = dv12[3];
    Theta1_d[0] = dv11[0];
    Theta1_d[1] = dv11[1];
    Theta1_d[2] = dv11[2];
    Theta1_d[3] = dv11[3];
    r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
    r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
    r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
    if (coder::c_norm(b_r_II_B) < 0.01) {
      reached_rest_centroid = 1U;
    }
    break;
  case 1:
    waypoint_toggle = false;
    //  rockback before step
    if (!coder::all(legs_valid)) {
      switch (reached_centroid) {
      case 0:
        //  hasn't started moving towards centroid yet
        b_r_II_B_d_temp[0] = r_II_B_d_temp[0];
        b_r_II_B_d_temp[1] = r_II_B_d_temp[1];
        b_r_II_B_d_temp[2] = r_II_B_d_temp[2];
        (void)std::copy(&T_I_B_d_temp[0], &T_I_B_d_temp[9], &b_dv8[0]);
        bv3[0] = floor_toggle[0];
        bv3[1] = floor_toggle[1];
        bv3[2] = floor_toggle[2];
        bv3[3] = floor_toggle[3];
        Body_Pose_Controller(r_II_c, b_dv8, b_r_II_B_d_temp, r_II_B, bv3, dv14,
                             dv15, dv16);
        Theta3_d[0] = dv16[0];
        Theta3_d[1] = dv16[1];
        Theta3_d[2] = dv16[2];
        Theta3_d[3] = dv16[3];
        Theta2_d[0] = dv15[0];
        Theta2_d[1] = dv15[1];
        Theta2_d[2] = dv15[2];
        Theta2_d[3] = dv15[3];
        Theta1_d[0] = dv14[0];
        Theta1_d[1] = dv14[1];
        Theta1_d[2] = dv14[2];
        Theta1_d[3] = dv14[3];
        r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
        r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
        r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
        reached_centroid = 2U;
        break;
      case 2:
        //  moving towards centroid
        r_II_B[0] = gamma_m[3] - r_II_B_d_temp[0];
        r_II_B[1] = gamma_m[4] - r_II_B_d_temp[1];
        r_II_B[2] = gamma_m[5] - r_II_B_d_temp[2];
        if (coder::c_norm(r_II_B) < 0.01) {
          reached_centroid = 1U;
        }
        break;
      case 1:
        //  step
        //  step leg
        b_r_II_B_d_temp[0] = r_II_B_d_temp[0];
        b_r_II_B_d_temp[1] = r_II_B_d_temp[1];
        b_r_II_B_d_temp[2] = r_II_B_d_temp[2];
        (void)std::copy(&T_I_B_d_temp[0], &T_I_B_d_temp[9], &b_dv10[0]);
        bv5[0] = floor_toggle[0];
        bv5[1] = floor_toggle[1];
        bv5[2] = floor_toggle[2];
        bv5[3] = floor_toggle[3];
        Body_Pose_Controller(r_II_c, b_dv10, b_r_II_B_d_temp, r_II_B, bv5, dv20,
                             dv21, dv22);
        Theta3_d[0] = dv22[0];
        Theta3_d[1] = dv22[1];
        Theta3_d[2] = dv22[2];
        Theta3_d[3] = dv22[3];
        Theta2_d[0] = dv21[0];
        Theta2_d[1] = dv21[1];
        Theta2_d[2] = dv21[2];
        Theta2_d[3] = dv21[3];
        Theta1_d[0] = dv20[0];
        Theta1_d[1] = dv20[1];
        Theta1_d[2] = dv20[2];
        Theta1_d[3] = dv20[3];
        r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
        r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
        r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
        if (((static_cast<int>(step_state)) == 1) &&
            ((static_cast<int>(reached_centroid)) == 1)) {
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &d_emlrtBCI);
          }
          Theta1_d[(static_cast<int>(leg_index)) - 1] = Theta1_d_midpt;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &e_emlrtBCI);
          }
          Theta2_d[(static_cast<int>(leg_index)) - 1] = Theta2_d_midpt;
          if (((static_cast<int>(leg_index)) < 1) ||
              ((static_cast<int>(leg_index)) > 4)) {
            rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                 &f_emlrtBCI);
          }
          Theta3_d[(static_cast<int>(leg_index)) - 1] = Theta3_d_midpt;
        } else {
          switch (step_state) {
          case 2:
            b_r_II_c_dstep[0] = r_II_c_dstep[0];
            b_r_II_c_dstep[1] = r_II_c_dstep[1];
            b_r_II_c_dstep[2] = r_II_c_dstep[2];
            dv29[0] = r_II_c_current[0];
            dv29[1] = r_II_c_current[1];
            dv29[2] = r_II_c_current[2];
            Leg_Controller(b_r_II_c_dstep, dv29, T_I_B, r_II_B, leg_index, &d20,
                           &d21, &d22);
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &j_emlrtBCI);
            }
            Theta1_d[(static_cast<int>(leg_index)) - 1] = d20;
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &k_emlrtBCI);
            }
            Theta2_d[(static_cast<int>(leg_index)) - 1] = d21;
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &l_emlrtBCI);
            }
            Theta3_d[(static_cast<int>(leg_index)) - 1] = d22;
            r_II_c_dstep[0] = b_r_II_c_dstep[0];
            r_II_c_dstep[1] = b_r_II_c_dstep[1];
            r_II_c_dstep[2] = b_r_II_c_dstep[2];
            break;
          case 3:
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &g_emlrtBCI);
            }
            Theta1_d[(static_cast<int>(leg_index)) - 1] = Theta1_d_reset;
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &h_emlrtBCI);
            }
            Theta2_d[(static_cast<int>(leg_index)) - 1] = Theta2_d_reset;
            if (((static_cast<int>(leg_index)) < 1) ||
                ((static_cast<int>(leg_index)) > 4)) {
              rtDynamicBoundsError(static_cast<int>(leg_index), 1, 4,
                                   &i_emlrtBCI);
            }
            Theta3_d[(static_cast<int>(leg_index)) - 1] = Theta3_d_reset;
            break;
          default:
            /* no actions */
            break;
          }
        }
        //              if step_error <= 0.03
        //                  reached_centroid = uint8(0);
        //                  reached_rest_centroid = uint8(0);
        //              end
        break;
      default:
        /* no actions */
        break;
      }
    } else {
      r_II_B_d_temp[0] = gamma_m[3];
      r_II_B_d_temp[1] = gamma_m[4];
      r_II_B_d_temp[2] = r_II_B_d[2];
      b_r_II_B_d_temp[0] = r_II_B_d_temp[0];
      b_r_II_B_d_temp[1] = r_II_B_d_temp[1];
      b_r_II_B_d_temp[2] = r_II_B_d_temp[2];
      (void)std::copy(&T_I_B_d_temp[0], &T_I_B_d_temp[9], &b_dv9[0]);
      bv4[0] = floor_toggle[0];
      bv4[1] = floor_toggle[1];
      bv4[2] = floor_toggle[2];
      bv4[3] = floor_toggle[3];
      Body_Pose_Controller(r_II_c, b_dv9, b_r_II_B_d_temp, r_II_B, bv4, dv17,
                           dv18, dv19);
      Theta3_d[0] = dv19[0];
      Theta3_d[1] = dv19[1];
      Theta3_d[2] = dv19[2];
      Theta3_d[3] = dv19[3];
      Theta2_d[0] = dv18[0];
      Theta2_d[1] = dv18[1];
      Theta2_d[2] = dv18[2];
      Theta2_d[3] = dv18[3];
      Theta1_d[0] = dv17[0];
      Theta1_d[1] = dv17[1];
      Theta1_d[2] = dv17[2];
      Theta1_d[3] = dv17[3];
      r_II_B_d_temp[0] = b_r_II_B_d_temp[0];
      r_II_B_d_temp[1] = b_r_II_B_d_temp[1];
      r_II_B_d_temp[2] = b_r_II_B_d_temp[2];
    }
    break;
  default:
    /* no actions */
    break;
  }
  floor_toggle_out[0] = floor_toggle[0];
  legs_valid_out[0] = legs_valid[0];
  floor_toggle_out[1] = floor_toggle[1];
  legs_valid_out[1] = legs_valid[1];
  floor_toggle_out[2] = floor_toggle[2];
  legs_valid_out[2] = legs_valid[2];
  floor_toggle_out[3] = floor_toggle[3];
  legs_valid_out[3] = legs_valid[3];
  // Theta_d = [Theta1_d,Theta2_d,Theta3_d]; % output
  //  Euler_d = [phi_d_temp;theta_d;psi_d];
  *phi_d_temp_out = phi_d_temp;
  r_II_B_d_temp_out[0] = r_II_B_d_temp[0];
  r_II_B_d_temp_out[1] = r_II_B_d_temp[1];
  r_II_B_d_temp_out[2] = r_II_B_d_temp[2];
  Theta1_d_out[0] = Theta1_d[0];
  Theta2_d_out[0] = Theta2_d[0];
  Theta3_d_out[0] = Theta3_d[0];
  Theta1_d_out[1] = Theta1_d[1];
  Theta2_d_out[1] = Theta2_d[1];
  Theta3_d_out[1] = Theta3_d[1];
  Theta1_d_out[2] = Theta1_d[2];
  Theta2_d_out[2] = Theta2_d[2];
  Theta3_d_out[2] = Theta3_d[2];
  Theta1_d_out[3] = Theta1_d[3];
  Theta2_d_out[3] = Theta2_d[3];
  Theta3_d_out[3] = Theta3_d[3];
}

//
// Controls Robot's walking algorithm
//    input: r_II_B_d, Euler_d, gamma_m
//    output: Theta_d (1-3), phi_d_temp & r_II_b_d_temp (orientation for
//    plotting), floor_toggle & legs_valid (for computing forces) NOTE: For the
//    real system, phi_d_temp & r_II_b_d_temp, floor_toggle & legs_valid can be
//    thrown away. For kinematic system, floor_toggle is not needed. For dynamic
//    system phi_d_temp % r_II_B_d_temp is not needed
//    r_II_B_d = [x_d;y_d;z_d];
//    Euler_d = [phi,theta,psi];
//    gamma_m = [Euler,r_II_B,Theta1,Theta2,Theta3];
//    Euler = [phi,theta,psi];
//    r_II_B = [x,y,z];
//    Theta1 = [Theta1FR,FL,BR,BL];
//    Theta2 = [Theta2FR,FL,BR,BL];
//    Theta3 = [Theta3FR,FL,BR,BL];
//
// Arguments    : void
// Return Type  : void
//
void Robot_Control_init()
{
  waypoint_toggle = false;
  turn_toggle = false;
  step_state = 0U;
  turn_state = false;
  reached_centroid = 0U;
  reached_rest_centroid = 1U;
  step_needed = 1U;
  calc_manip = true;
  legs_valid[0] = true;
  legs_valid[1] = true;
  legs_valid[2] = true;
  legs_valid[3] = true;
  legs_stepped = 0U;
  leg_reset_needed = false;
  coder::eye(T_I_B_d_temp);
  phi_d_temp = 0.0;
  is_turning = 0U;
  leg_index = 0U;
  Theta1_d_midpt = 0.0;
  Theta2_d_midpt = 0.0;
  Theta3_d_midpt = 0.0;
  Theta1_d_reset = 0.0;
  Theta2_d_reset = 0.0;
  Theta3_d_reset = 0.0;
  r_II_c_BL_0[0] = 0.0;
  r_II_c_BR_0[0] = 0.0;
  r_II_c_FL_0[0] = 0.0;
  r_II_c_FR_0[0] = 0.0;
  r_II_B_d_temp[0] = 0.0;
  r_II_c_dstep[0] = 0.0;
  r_II_c_current[0] = 0.0;
  r_II_c_BL_0[1] = 0.0;
  r_II_c_BR_0[1] = 0.0;
  r_II_c_FL_0[1] = 0.0;
  r_II_c_FR_0[1] = 0.0;
  r_II_B_d_temp[1] = 0.0;
  r_II_c_dstep[1] = 0.0;
  r_II_c_current[1] = 0.0;
  r_II_c_BL_0[2] = 0.0;
  r_II_c_BR_0[2] = 0.0;
  r_II_c_FL_0[2] = 0.0;
  r_II_c_FR_0[2] = 0.0;
  r_II_B_d_temp[2] = 0.0;
  r_II_c_dstep[2] = 0.0;
  r_II_c_current[2] = 0.0;
  Theta1_d[0] = 0.0;
  Theta2_d[0] = 0.0;
  Theta3_d[0] = 0.0;
  manip_vec[0] = 0.0;
  Theta1_d[1] = 0.0;
  Theta2_d[1] = 0.0;
  Theta3_d[1] = 0.0;
  manip_vec[1] = 0.0;
  Theta1_d[2] = 0.0;
  Theta2_d[2] = 0.0;
  Theta3_d[2] = 0.0;
  manip_vec[2] = 0.0;
  Theta1_d[3] = 0.0;
  Theta2_d[3] = 0.0;
  Theta3_d[3] = 0.0;
  manip_vec[3] = 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
void endPhi_not_empty_init()
{
  endPhi_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void endPoint_not_empty_init()
{
  endPoint_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void floor_toggle_not_empty_init()
{
  floor_toggle_not_empty = false;
}

} // namespace Codegen

//
// File trailer for Robot_Control.cpp
//
// [EOF]
//
