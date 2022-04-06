#include "vision.hpp"
#include <math.h>
/*
%**************************************************************************
% R. Munguia 2020
% 
% Inverse projection model hc = [hc1 hc2 hc3]' = h^(-1)([u v]')
%
% Output parameters: 
% 1) Return a vector "hc" pointing in the direction of the undistorted image
% point, expressed in the camera frame.   data type:  arma::vec::fixed<3>
% 2) dhc_duv = Jacobian   data type : arma::mat::fixed<3,2>
%
% Input parameters:
% 1) uvd,  distorted pixel coordinates of the image point [ud vd]
% 2) model_type, projective model -> 1 , omnidirectional model -> 2
% 3) return_Jac,  without jacobian -> true , with jacobian -> false
% 
% 5) camera_parameters struct: 
% for projective camera:  
struct CAM
{
  double distortions[5];
  double cc[2];
  double fc[2];
  double alpha_c;  
};
% for omnidirectional camera:  kc = cam_parameters.Omni.distortions;
%                              XI = cam_parameters.Omni.xi;
%                              gamma = cam_parameters.Omni.gamma;
%                              CC = cam_parameters.Omni.CC;
%                              alpha_c = cam_parameters.Omni.alpha_c;
%
*/

arma::vec::fixed<3> Inverse_projection_model(cv::Point2d uvd,int model_type,bool return_Jac,CAM camera_parameters, arma::mat::fixed<3,2>& dhc_duvd )
{
    double fc1 = camera_parameters.fc[0];
    double fc2 = camera_parameters.fc[1];
    double cc1 = camera_parameters.cc[0];
    double cc2 = camera_parameters.cc[1];


    cv::Point2d uv;
    int distortion_model = 1;
    uv = Undistort_a_point( uvd,camera_parameters,distortion_model );  // undistort point

    arma::vec::fixed<3> hc;

    hc(0) = (uv.x - cc1 )/fc1;
    hc(1) = (uv.y - cc2 )/fc2;
    hc(2) = 1;

    if (return_Jac == true) // compute Jacobian
    {
      arma::mat::fixed<3,2> dhc_duuvu = { {1/fc1,  0 },
                                          {0,    1/fc2},                                         
                                          {0,      0}};

      arma::mat::fixed<2,2> Jdp = Jac_Distort_a_point(uv,camera_parameters,distortion_model);

      dhc_duvd = dhc_duuvu*arma::inv(Jdp);

      
    }

 

return hc;
}

/*

%**************************************************************************
% R. Munguia 2020
% 
% Projection of a point "Pc" (defined in camera coordinates) to the image plane [u  v]' 
%
% Output parameters: 
% 1) [u v] (distorted) are in pixel coordiantes
%  2) duv_dPc = Jacobian
% Input parameters:
% 1) Pc,  Point [x y z]' in euclidean coordinates expressed in camera
% coordinate frame
% 2) type, projective model -> 1 , omnidirectional model -> 2
% 3) Jac,  without jacobian -> false , with jacobian -> true
% 5) Camera parameters: 
struct CAM
{
  double distortions[5];
  double cc[2];
  double fc[2];
  double alpha_c;  
};
% for omnidirectional camera:  kc = cam_parameters.Omni.distortions;
%                              XI = cam_parameters.Omni.xi;
%                              gamma = cam_parameters.Omni.gamma;
%                              CC = cam_parameters.Omni.CC;
%                              alpha_c = cam_parameters.Omni.alpha_c;
%
*/
cv::Point2d Projection_model(arma::vec::fixed<3> Pc,int model_type,bool return_Jac,CAM camera_parameters, arma::mat::fixed<2,3>& duv_dPc)
{

    double fc1 = camera_parameters.fc[0];
    double fc2 = camera_parameters.fc[1];
    double cc1 = camera_parameters.cc[0];
    double cc2 = camera_parameters.cc[1];
    double alpha_c = camera_parameters.alpha_c;

    cv::Point2d uvd;

    if (Pc(2) > 0) // if the point is not behind the camera
    {
      arma::mat::fixed<3,3>  KK = {{fc1, alpha_c*fc2, cc1},
                                   { 0 ,    fc2  ,    cc2},
                                   { 0 ,      0  ,    1  }};
     arma::vec::fixed<3> ph = KK*Pc;

     cv::Point2d uv;

     uv.x = ph(0)/ph(2);   // undistorted image coordinates
     uv.y = ph(1)/ph(2);   

     int ditortion_model = 1; 
     uvd = Distort_a_point(uv,camera_parameters,ditortion_model);


    }
    else
    {
      uvd.x = -1;
      uvd.y = -1;
    }

    if (return_Jac == true)
    {
      int distortion_model = 1; 
      arma::mat::fixed<2,2> dh_uuud = Jac_Distort_a_point(uvd,camera_parameters,distortion_model);
      double Pc1 = Pc(0);
      double Pc2 = Pc(1);
      double Pc3 = Pc(2);
      
      arma::mat::fixed<2,3> dhuuvu_Pc = {{ fc1/Pc3, (alpha_c*fc2)/Pc3, cc1/Pc3 - (Pc3*cc1 + Pc1*fc1 + Pc2*alpha_c*fc2)/pow(Pc3,2)},
                                        { 0, fc2/Pc3, cc2/Pc3 - (Pc3*cc2 + Pc2*fc2)/pow(Pc3,2)}};

                    


      duv_dPc = dh_uuud*dhuuvu_Pc;

    }
    


  return uvd;
}



//----------------------------------------------------------------------------------------------------
// Function for compute the Jacobian of the Distort_a_point function
// Rodrigo Munguia 2020
// 1  --> 
//       Bouguets Calibration Toolbox for MATLAB® (Bouguet 2010)        
//       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
//       % WITHOUT tangencial distortion
//
// 2  --> to be implemented
arma::mat::fixed<2,2> Jac_Distort_a_point(cv::Point2d uv, CAM camera_parameters,int distortion_model )
{
  //-------------------------------------------------------
   if(distortion_model == 1) // (Bouguet 2010)   
   { 
     double k1 = camera_parameters.distortions[0];
     double k2 = camera_parameters.distortions[1];
     double fc1 = camera_parameters.fc[0];
     double fc2 = camera_parameters.fc[1];
     double cc1 = camera_parameters.cc[0];
     double cc2 = camera_parameters.cc[1];

    double  um = (uv.x-cc1)/fc1;
    double vm = (uv.y-cc2)/fc2;
    double r = sqrt(um*um + vm*vm);
    double g = (1 + k1*r*r + k2*pow(r,4));

    double dg_d_r = 4*k2*pow(r,3) + 2*k1*r;

    arma::mat::fixed<2,2> duvd_duvmd  = { {fc1,  0 },
                                        {0,    fc2}};                               
    
    arma::vec::fixed<2> uvm = {um,vm};
    
    arma::rowvec::fixed<2> dr_duvm = { um/r, vm/r} ;  


    arma::mat::fixed<2,2> duvmd_duvm = arma::eye(2,2)*g + uvm*dg_d_r*dr_duvm;

    arma::mat::fixed<2,2> duvm_uv  = { {1/fc1,  0 },
                                        {0,    1/fc2}};    

    arma::mat::fixed<2,2> duv_uvd = duvd_duvmd*duvmd_duvm*duvm_uv;

    return duv_uvd;

   }  


}

//-----------------------------------------------------------------------------------------------------
// Undistort a distorted pointe
// distortion_model:
// 1  --> 
//       Bouguets Calibration Toolbox for MATLAB® (Bouguet 2010)        
//       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
//       % WITHOUT tangencial distortion
//
// 2  --> to be implemented
cv::Point2d Distort_a_point(cv::Point2d uv,CAM camera_parameters,int ditortion_model)
{
  cv::Point2d uvd;
  //-------------------------------------------------------
   if(ditortion_model == 1) // (Bouguet 2010)   
   { 
     double k1 = camera_parameters.distortions[0];
     double k2 = camera_parameters.distortions[1];
     double fc1 = camera_parameters.fc[0];
     double fc2 = camera_parameters.fc[1];
     double cc1 = camera_parameters.cc[0];
     double cc2 = camera_parameters.cc[1];

     double um = (uv.x-cc1)/fc1;
     double vm = (uv.y-cc2)/fc2;
     
     double r = sqrt(um*um + vm*vm);
     
     double g = (1 + k1*r*r + k2*pow(r,4));

     double umd = um*g;
     double vmd = vm*g;

     

     uvd.x = fc1*umd + cc1;
     uvd.y = fc2*vmd + cc2;

   

   }  

  return uvd;
}



//-----------------------------------------------------------------------------------------------------
// Undistort a distorted pointe
// distortion_model:
// 1  --> 
//       Bouguets Calibration Toolbox for MATLAB® (Bouguet 2010)        
//       % http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html 
//       % WITHOUT tangencial distortion
//
// 2  --> to be implemented
cv::Point2d Undistort_a_point(cv::Point2d uvd,CAM camera_parameters,int ditortion_model)
{ 
  cv::Point2d uv;  // undistorted point
   
   //-------------------------------------------------------
   if(ditortion_model == 1) // (Bouguet 2010)   
   { 
     double k1 = camera_parameters.distortions[0];
     double k2 = camera_parameters.distortions[1];
     double fc1 = camera_parameters.fc[0];
     double fc2 = camera_parameters.fc[1];
     double cc1 = camera_parameters.cc[0];
     double cc2 = camera_parameters.cc[1];
     

     double ud = uvd.x;
     double vd = uvd.y;

    // First: Subtract principal point, and divide by the focal length:
     double   um = (ud-cc1)/fc1;
     double   vm = (vd-cc2)/fc2;
    
    double x[2] = {um,vm};

    for (int i= 1;i<20;i++)
    {
      double r_2 = x[0]*x[0] + x[1]*x[1];
      double k_radial = 1 + k1*r_2 + k2*r_2*r_2;
      x[0] = um/k_radial;
      x[1] = vm/k_radial;
    } 

    uv.x = fc1*x[0] + cc1;
    uv.y = fc2*x[1] + cc2;
     
   }
   //------------------------------------------

return uv;
}
