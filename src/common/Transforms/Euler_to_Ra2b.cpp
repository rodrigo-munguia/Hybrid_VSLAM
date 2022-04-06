//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Euler_to_Ra2b_colum_major.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 30-Aug-2020 14:29:18
//

// Include Files
#include "Euler_to_Ra2b.hpp"
#include <cmath>

// Function Definitions

//
// **************************************************************************
//  Rodrigo Munguía 2012
//  Function for transforming from EULER angles to rotation matrix (Ra2b)
//  The rotation is defined by the sequence of rotations (of the body respect to the global reference)
// :  yaw -> pitch -> roll   or  R =  R(roll)R(pitch)R(yaw)
// Tait-Bryan Angles     (Small Unmmaned Aircraft eq. 2.5)
  
//
//  Body to world rotation matrix "Rb2w" is obtained if the orientation of the body are determined by (euler angles) extrinsic rotations
//  [Rb2w] = Euler_to_Rb2w (roll,pitch,yaw)
// Arguments    : double roll
//                double pitch
//                double yaw
//                double Ra2b[9]
// Return Type  : void
//
//  World to body rotation matrix "Rw2b" is obtained if the orientation of the body are determined by (euler angles) intrinsic rotations 
  //  [Rw2b] = Euler_to_Rb2w (roll,pitch,yaw)
  //  world (w) =  navigation (n)
  //  body (n) = robot (r)
  // **************************************************************************
  // Euler angles
void Euler_to_Ra2b_colum_major(double roll, double pitch, double yaw, double Ra2b[9])
{
  double Ra2b_tmp;
  double b_Ra2b_tmp;
  double c_Ra2b_tmp;
  double d_Ra2b_tmp;
  double e_Ra2b_tmp;
  double f_Ra2b_tmp;   
  
  Ra2b_tmp = std::cos(pitch);
  b_Ra2b_tmp = std::sin(yaw);
  c_Ra2b_tmp = std::cos(yaw);
  d_Ra2b_tmp = std::sin(pitch);
  e_Ra2b_tmp = std::cos(roll);
  f_Ra2b_tmp = std::sin(roll);
  
  // colum major order
  Ra2b[0] = c_Ra2b_tmp * Ra2b_tmp;
  Ra2b[3] = b_Ra2b_tmp * Ra2b_tmp;
  Ra2b[6] = -d_Ra2b_tmp;
  Ra2b[1] = -b_Ra2b_tmp * e_Ra2b_tmp + c_Ra2b_tmp * d_Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[4] = c_Ra2b_tmp * e_Ra2b_tmp + b_Ra2b_tmp * d_Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[7] = Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[2] = b_Ra2b_tmp * f_Ra2b_tmp + std::cos(yaw) * std::sin(pitch) * e_Ra2b_tmp;
  Ra2b[5] = -c_Ra2b_tmp * f_Ra2b_tmp + std::sin(yaw) * std::sin(pitch) *e_Ra2b_tmp;
  Ra2b[8] = Ra2b_tmp * e_Ra2b_tmp;
}

void Euler_to_Ra2b_row_major(double roll, double pitch, double yaw, double Ra2b[9])
{
  double Ra2b_tmp;
  double b_Ra2b_tmp;
  double c_Ra2b_tmp;
  double d_Ra2b_tmp;
  double e_Ra2b_tmp;
  double f_Ra2b_tmp;   
  
  Ra2b_tmp = std::cos(pitch);
  b_Ra2b_tmp = std::sin(yaw);
  c_Ra2b_tmp = std::cos(yaw);
  d_Ra2b_tmp = std::sin(pitch);
  e_Ra2b_tmp = std::cos(roll);
  f_Ra2b_tmp = std::sin(roll);
  
  // row major order
  Ra2b[0] = c_Ra2b_tmp * Ra2b_tmp;
  Ra2b[1] = b_Ra2b_tmp * Ra2b_tmp;
  Ra2b[2] = -d_Ra2b_tmp;
  Ra2b[3] = -b_Ra2b_tmp * e_Ra2b_tmp + c_Ra2b_tmp * d_Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[4] = c_Ra2b_tmp * e_Ra2b_tmp + b_Ra2b_tmp * d_Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[5] = Ra2b_tmp * f_Ra2b_tmp;
  Ra2b[6] = b_Ra2b_tmp * f_Ra2b_tmp + std::cos(yaw) * std::sin(pitch) * e_Ra2b_tmp;
  Ra2b[7] = -c_Ra2b_tmp * f_Ra2b_tmp + std::sin(yaw) * std::sin(pitch) *e_Ra2b_tmp;
  Ra2b[8] = Ra2b_tmp * e_Ra2b_tmp;
}




void Ra2b_to_Euler_colum_major(double &roll, double &pitch, double &yaw, double Ra2b[9])
{
  // For rotations:  yaw -> pitch -> roll   or  R =  R(roll)R(pitch)R(yaw)
  // Ra2b[9] is in column major
  double R11 = Ra2b[0];
  double R21 = Ra2b[1];
  double R31 = Ra2b[2];

  double R12 = Ra2b[3];
  double R22 = Ra2b[4];
  double R32 = Ra2b[5];

  double R13 = Ra2b[6];
  double R23 = Ra2b[7];
  double R33 = Ra2b[8];

  roll = std::atan2(-R23,R33);

  pitch = std::atan2(R13, std::pow(1 - R13*R13,.5 ) );

  yaw = std::atan2(-R12,R11);

}
//
// File trailer for Euler_to_Ra2b_colum_major.cpp
//
// [EOF]
//
