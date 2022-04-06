//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Ra2b_TO_Quat_a2b.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Aug-2020 11:48:55
//
#ifndef RA2B_TO_QUAT_A2B_H
#define RA2B_TO_QUAT_A2B_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include <armadillo>
//#include "rtwtypes.h"
//#include "Ra2b_TO_Quat_a2b_types.h"

// Function Declarations
extern void Ra2b_TO_Quat_a2b(const double Ra2b[9], double q_a2b[4]);

void Ra2b_TO_Quat_a2b_arma(arma::mat::fixed<3,3> &Ra2b, arma::vec::fixed<4> &q_a2b  );

#endif

//
// File trailer for Ra2b_TO_Quat_a2b.h
//
// [EOF]
//
