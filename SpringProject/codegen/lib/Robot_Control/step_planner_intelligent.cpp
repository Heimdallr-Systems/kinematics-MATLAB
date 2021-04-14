//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: step_planner_intelligent.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Apr-2021 15:32:06
//

// Include Files
#include "step_planner_intelligent.h"
#include "isequal.h"
#include "norm.h"

// Function Definitions
//
// This function returns r_II_c for all 4 joint positions, given a direction
//  of travel, starting location and step distance.
//  Rewritten to take into account the point cloud of the floor for placing
//  the foot in a more safe location.
//  INPUTS:
//  r_II_b: Current location of the robot's body
//  r_II_b_last: previous location of the robot's body
//  r_II_c_start: current location of the robot's contact points in the I
//  frame
//  leg_index: index of the leg being moved
//  step_dist: Linear distance a contact point will travel in a step
//  floor_map: The point cloud of the robot's immediate surroundings
//  step_tolerance: range at which a step location can deviate
//  OUTPUTS:
//  r_II_c: updated location of the contact points of the robot's legs.
//
// Arguments    : const double r_II_b_last[3]
//                const double r_II_b[3]
//                const double r_II_c_start[3]
//                double step_dist
//                double r_II_c[3]
// Return Type  : void
//
namespace Codegen {
void step_planner_intelligent(const double r_II_b_last[3],
                              const double r_II_b[3],
                              const double r_II_c_start[3], double step_dist,
                              double r_II_c[3])
{
  double b_dv[3];
  double travel_dir_0[3];
  double d;
  //  CONSTANTS
  //  point cloud resolution, points/mm
  //  c_pc_res=.1;
  //  convert travel_dir to a unit vector if it is not already.
  travel_dir_0[0] = r_II_b[0] - r_II_b_last[0];
  b_dv[0] = 0.0;
  travel_dir_0[1] = r_II_b[1] - r_II_b_last[1];
  b_dv[1] = 0.0;
  travel_dir_0[2] = r_II_b[2] - r_II_b_last[2];
  b_dv[2] = 0.0;
  if (coder::b_isequal(travel_dir_0, b_dv)) {
    travel_dir_0[0] = r_II_b[0] - r_II_c_start[0];
    travel_dir_0[1] = r_II_b[1] - r_II_c_start[1];
    travel_dir_0[2] = r_II_b[2] - r_II_c_start[2];
  }
  d = coder::c_norm(travel_dir_0);
  r_II_c[0] = r_II_c_start[0] + ((travel_dir_0[0] / d) * step_dist);
  r_II_c[1] = r_II_c_start[1] + ((travel_dir_0[1] / d) * step_dist);
  r_II_c[2] = r_II_c_start[2] + ((travel_dir_0[2] / d) * step_dist);
  r_II_c[2] -= 0.005;
  //  r_BB_c_target = r_II_c(:,leg_index)-r_II_b;
  //  determine center coordinates
  //  ideal_leg_pos=round(r_BB_c_target*c_pc_res);
}

} // namespace Codegen

//
// File trailer for step_planner_intelligent.cpp
//
// [EOF]
//
