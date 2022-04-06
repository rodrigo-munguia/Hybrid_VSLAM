//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Euler_to_Ra2b_colum_major.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 30-Aug-2020 14:29:18
//
#ifndef Euler_to_Ra2b_colum_major_H
#define Euler_to_Ra2b_colum_major_H

// Include Files
#include <cstddef>
#include <cstdlib>
//#include "rtwtypes.h"
//#include "Euler_to_Ra2b_colum_major_types.h"

// Function Declarations
void Euler_to_Ra2b_colum_major(double roll, double pitch, double yaw, double Ra2b[9]);
void Euler_to_Ra2b_row_major(double roll, double pitch, double yaw, double Ra2b[9]);

void Ra2b_to_Euler_colum_major(double &roll, double &pitch, double &yaw, double Ra2b[9]);

#endif

//
// File trailer for Euler_to_Ra2b_colum_major.h
//
// [EOF]
//
