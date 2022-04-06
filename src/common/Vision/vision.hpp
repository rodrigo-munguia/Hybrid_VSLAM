#ifndef VISION_I_H
#define VISION_I_H


#include <armadillo>
#include "opencv2/opencv.hpp"


//--- camera parameters structure


struct CAM
{
  double *distortions;
  double cc[2];
  double fc[2];
  double alpha_c;  
};


//-----------------------------------------------------------------------------
//   Initialize visual features with range measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

arma::vec::fixed<3> Inverse_projection_model(cv::Point2d uvd,int model_type,bool return_Jac,CAM camera_parameters, arma::mat::fixed<3,2>& dhc_duvd );

cv::Point2d Projection_model(arma::vec::fixed<3> Pc,int model_type,bool return_Jac,CAM camera_parameters, arma::mat::fixed<2,3>& duv_dPc);

cv::Point2d Undistort_a_point(cv::Point2d uvd,CAM camera_parameters,int ditortion_model);

cv::Point2d Distort_a_point(cv::Point2d uv,CAM camera_parameters,int ditortion_model);

arma::mat::fixed<2,2> Jac_Distort_a_point(cv::Point2d uv, CAM camera_parameters,int distortion_model );

#endif