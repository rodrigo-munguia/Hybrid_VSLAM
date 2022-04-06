//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Ra2b_TO_Quat_a2b.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Aug-2020 11:48:55
//

// Include Files
#include "Ra2b_TO_Quat_a2b.hpp"
#include <cmath>

// Function Definitions

//
// R_n_to_b  to Quaternion b  %D.15
// Arguments    : const double Ra2b[9]
//                double q_a2b[4]
// Return Type  : void
//



void Ra2b_TO_Quat_a2b(const double Ra2b[9], double q_a2b[4])
{ 
  // Column major
  double b1;
  b1 = 0.5 * std::sqrt(((Ra2b[0] + 1.0) + Ra2b[4]) + Ra2b[8]);
  q_a2b[0] = b1;                                // w
  q_a2b[1] = (Ra2b[5] - Ra2b[7]) / (4.0 * b1); // x
  q_a2b[2] = (Ra2b[6] - Ra2b[2]) / (4.0 * b1);  // y
  q_a2b[3] = (Ra2b[1] - Ra2b[3]) / (4.0 * b1);  // z

  //  D.15
  //  *************************
}


void Ra2b_TO_Quat_a2b_arma(arma::mat::fixed<3,3> &Ra2b, arma::vec::fixed<4> &q_a2b  )
{
  // Row 


  double b1;
  b1 = 0.5 * std::sqrt(((Ra2b(0,0) + 1.0) + Ra2b(1,1)) + Ra2b(2,2));
  q_a2b(0) = b1;                                    // w
  q_a2b(1) = (Ra2b(2,1) - Ra2b(1,2)) / (4.0 * b1);  // x
  q_a2b(2) = (Ra2b(0,2) - Ra2b(2,0)) / (4.0 * b1);  // y
  q_a2b(3) = (Ra2b(1,0) - Ra2b(0,1)) / (4.0 * b1);  // z


}
//
// File trailer for Ra2b_TO_Quat_a2b.cpp
//
// [EOF]
//
