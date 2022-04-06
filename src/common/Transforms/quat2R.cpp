//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quat2R.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 03-Sep-2020 10:34:36
//

// Include Files
#include "quat2R.hpp"
#include <cmath>
#include <iostream>

// Function Definitions

//
// Arguments    : double b[4]
//                double Rn2b[9]
// Return Type  : void
//
//

void quat2R(double b[4], double Rn2b[9])
{
  double scale;
  double absxk;
  double t;
  double d;
  double Bv[3];
  double a[9];
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  //  convert a quaternion b to a rotation matrix
  scale = 3.3121686421112381E-170;
  absxk = std::abs(b[0]);
  if (absxk > 3.3121686421112381E-170) {
    d = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    d = t * t;
  }

  absxk = std::abs(b[1]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  absxk = std::abs(b[2]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  absxk = std::abs(b[3]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  d = scale * std::sqrt(d);
  if (d != 0.0) {
    int i;
    b[0] /= d;
    b[1] /= d;
    b[2] /= d;
    b[3] /= d;
    Bv[0] = b[1];
    Bv[1] = b[2];
    Bv[2] = b[3];
    scale = b[0] * b[0] - ((b[1] * b[1] + b[2] * b[2]) + b[3] * b[3]);
    absxk = 2.0 * b[0];
    for (i = 0; i < 3; i++) {
      int Rn2b_tmp;
      Rn2b[3 * i] = scale * static_cast<double>(b_b[3 * i]) + 2.0 * Bv[0] * Bv[i];
      Rn2b_tmp = 3 * i + 1;
      Rn2b[Rn2b_tmp] = scale * static_cast<double>(b_b[Rn2b_tmp]) + 2.0 * Bv[1] *
        Bv[i];
      Rn2b_tmp = 3 * i + 2;
      Rn2b[Rn2b_tmp] = scale * static_cast<double>(b_b[Rn2b_tmp]) + 2.0 * Bv[2] *
        Bv[i];
    }

    a[0] = absxk * 0.0;
    a[3] = absxk * -b[3];
    a[6] = absxk * b[2];
    a[1] = absxk * b[3];
    a[4] = absxk * 0.0;
    a[7] = absxk * -b[1];
    a[2] = absxk * -b[2];
    a[5] = absxk * b[1];
    a[8] = absxk * 0.0;
    for (i = 0; i < 9; i++) {
      Rn2b[i] += a[i];
     // std::cout << Rn2b[i] << std::endl;
    }
    //int  q = 10;
    //  Rn2b1 = [(b(1)^2+b(2)^2-b(3)^2-b(4)^2) 2*(b(2)*b(3)-b(1)*b(4)) 2*(b(1)*b(3)+b(2)*b(4)) 
    //          2*(b(2)*b(3)+b(1)*b(4)) (b(1)^2-b(2)^2+b(3)^2-b(4)^2) 2*(b(3)*b(4)-b(1)*b(2)) 
    //          2*(b(2)*b(4)-b(1)*b(3)) 2*(b(1)*b(2)+b(3)*b(4))   b(1)^2-b(2)^2-b(3)^2+b(4)^2] 
  } else {
    //  fault condition
  }
}

//
// File trailer for quat2R.cpp
//
// [EOF]
//

void quat2R_row_major(double b[4], double Rn2b[9])
{

  double scale;
  double absxk;
  double t;
  double d;
  double Bv[3];
  double a[9];
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  //  convert a quaternion b to a rotation matrix
  scale = 3.3121686421112381E-170;
  absxk = std::abs(b[0]);
  if (absxk > 3.3121686421112381E-170) {
    d = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    d = t * t;
  }

  absxk = std::abs(b[1]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  absxk = std::abs(b[2]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  absxk = std::abs(b[3]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  d = scale * std::sqrt(d);
  if (d != 0.0) {
    int i;
    b[0] /= d;
    b[1] /= d;
    b[2] /= d;
    b[3] /= d;
    Bv[0] = b[1];
    Bv[1] = b[2];
    Bv[2] = b[3];
    scale = b[0] * b[0] - ((b[1] * b[1] + b[2] * b[2]) + b[3] * b[3]);
    absxk = 2.0 * b[0];
    for (i = 0; i < 3; i++) {
      int Rn2b_tmp;
      Rn2b[3 * i] = scale * static_cast<double>(b_b[3 * i]) + 2.0 * Bv[0] * Bv[i];
      Rn2b_tmp = 3 * i + 1;
      Rn2b[Rn2b_tmp] = scale * static_cast<double>(b_b[Rn2b_tmp]) + 2.0 * Bv[1] *
        Bv[i];
      Rn2b_tmp = 3 * i + 2;
      Rn2b[Rn2b_tmp] = scale * static_cast<double>(b_b[Rn2b_tmp]) + 2.0 * Bv[2] *
        Bv[i];
    }

    a[0] = absxk * 0.0;
    a[1] = absxk * -b[3];
    a[2] = absxk * b[2];
    a[3] = absxk * b[3];
    a[4] = absxk * 0.0;
    a[5] = absxk * -b[1];
    a[6] = absxk * -b[2];
    a[7] = absxk * b[1];
    a[8] = absxk * 0.0;
    for (i = 0; i < 9; i++) {
      Rn2b[i] += a[i];
     // std::cout << Rn2b[i] << std::endl;
    }
    //int  q = 10;
    //  Rn2b1 = [(b(1)^2+b(2)^2-b(3)^2-b(4)^2) 2*(b(2)*b(3)-b(1)*b(4)) 2*(b(1)*b(3)+b(2)*b(4)) 
    //          2*(b(2)*b(3)+b(1)*b(4)) (b(1)^2-b(2)^2+b(3)^2-b(4)^2) 2*(b(3)*b(4)-b(1)*b(2)) 
    //          2*(b(2)*b(4)-b(1)*b(3)) 2*(b(1)*b(2)+b(3)*b(4))   b(1)^2-b(2)^2-b(3)^2+b(4)^2] 
  } else {
    //  fault condition
  }
}

//
// File trailer for quat2R.cpp
//
// [EOF]
//




