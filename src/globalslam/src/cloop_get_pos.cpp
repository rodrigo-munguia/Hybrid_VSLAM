#include "cloop.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"



// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

bool CLOOP::Get_pos_of_current_frame(arma::vec::fixed<3> &pos,KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points)
{

    vector<cv::Point3f> Points3D;
    vector<cv::Point2f> Points2D;

    for (int i = 0; i < idx_pt_matches.size();i++)
    {
        int64 idx_pt = idx_pt_matches[i];

        double x = Gmap.AnchorsDATA[idx_pt].AnchorState[0]; 
        double y = Gmap.AnchorsDATA[idx_pt].AnchorState[1];
        double z = Gmap.AnchorsDATA[idx_pt].AnchorState[2];

        Points3D.push_back(cv::Point3f(x, y, z)); 

        cv::Point2d uv;
        uv = Undistort_a_point(image_points[i],cam_parameters,1);
        //uv = Distort_a_point(image_points[i],cam_parameters,1);

        Points2D.push_back(cv::Point2f(uv.x, uv.y)); 

    }

/*
    void PnPProblem::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
                                     const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
                                     int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
                                     float reprojectionError, double confidence )    // Ransac parameters
*/


//double Rn2c[9] = {kf_cl.Rn2c.at(0,0), kf_cl.Rn2c.at(1,0), kf_cl.Rn2c.at(2,1) , kf_cl.Rn2c.at(0,1), kf_cl.Rn2c.at(1,1), kf_cl.Rn2c.at(2,1), kf_cl.Rn2c.at(0,2), kf_cl.Rn2c.at(1,2), kf_cl.Rn2c.at(2,2)    };
//double roll,pitch,yaw;
//Ra2b_to_Euler_colum_major(roll, pitch, yaw, Rn2c);
//double rvec_ini[3] = {roll,pitch,yaw}; // consider ned to xyz convertion
//cv::Mat rvec = cv::Mat(3, 1, CV_64FC1,rvec_ini); 

arma::mat Rn2c_t = kf_cl.Rn2c.t(); // for converting to OpenCV mat    
cv::Mat R = to_cvmat(Rn2c_t); // Kf_1 Projection matrix (OpenCv format)

cv::Mat rvec;
cv::Rodrigues(R, rvec);
//cout << rvec_r << endl;

double tvec_ini[3] = {kf_cl.t_c2n[0],kf_cl.t_c2n[1],kf_cl.t_c2n[2] }; // consider ned to xyz convertion
cv::Mat tvec = cv::Mat(3, 1, CV_64FC1,tvec_ini); 


cv::Mat A_matrix_;
A_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
    A_matrix_.at<double>(0, 0) = cam_parameters.fc[0];       //      [ fx   0  cx ]
    A_matrix_.at<double>(1, 1) = cam_parameters.fc[1];       //      [  0  fy  cy ]
    A_matrix_.at<double>(0, 2) = cam_parameters.cc[0];       //      [  0   0   1 ]
    A_matrix_.at<double>(1, 2) = cam_parameters.cc[1];
    A_matrix_.at<double>(2, 2) = 1;

cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
cv::Mat inliers;

bool useExtrinsicGuess = true;   // if true the function uses the provided rvec and tvec values as
    // initial approximations of the rotation and translation vectors

 // RANSAC parameters
    int iterationsCount = 500;      // number of Ransac iterations.
    float reprojectionError = 1.0;  // maximum allowed distance to consider it an inlier.
    double confidence = 0.99;       // ransac successful confidence.

  int pnpMethod = cv::SOLVEPNP_ITERATIVE;
  //int pnpMethod =cv::SOLVEPNP_EPNP; 
      

  
  //cout << "intial att: " << rvec << endl;
  cv::Mat tvec_u = cv::Mat::zeros(3, 1, CV_64FC1);
  tvec_u.at<double>(0) = tvec.at<double>(0);
  tvec_u.at<double>(1) = tvec.at<double>(1);
  tvec_u.at<double>(2) = tvec.at<double>(2);
  cv::Mat rvec_u = cv::Mat::zeros(3, 1, CV_64FC1);
  rvec_u.at<double>(0) = rvec.at<double>(0);
  rvec_u.at<double>(1) = rvec.at<double>(1);
  rvec_u.at<double>(2) = rvec.at<double>(2);

  
  cv::solvePnPRansac( Points3D, Points2D, A_matrix_, distCoeffs, rvec, tvec,
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                        inliers, pnpMethod );
  
 //cv::solvePnP(Points3D, Points2D, A_matrix_, distCoeffs, rvec, tvec,useExtrinsicGuess,pnpMethod);

   // test --------------------

  if(!inliers.empty())
  {  
    double sum_e = 0;    
    double n_in = 0;
    
    for(int i = 0; i < Points3D.size(); i++)
      {   
          if(inliers.at<char>(i) != 0) 
            {
              arma::vec::fixed<3> Pt;
              Pt[0] = Points3D[i].x;
              Pt[1] = Points3D[i].y; 
              Pt[2] = Points3D[i].z;
              arma::vec::fixed<3> Tn2c;
              Tn2c[0] = tvec.at<double>(0);
              Tn2c[1] = tvec.at<double>(1);
              Tn2c[2] = tvec.at<double>(2);
              arma::vec::fixed<3> pc = kf_cl.Rn2c*(Pt - Tn2c); // feature point expressed in the camera frame
              arma::mat::fixed<2,3> duv_dPc;
              cv::Point2d uvd = Projection_model(pc,1,false,cam_parameters,duv_dPc );
              int sm =  0;
              // check if the Anchor is predicted to appear in the image 
              cv::Point2d uvm = Points2D[i];
              /*
              if((uvd.x > sm)&&(uvd.x < PAR.Mono_cam_img_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.Mono_cam_img_rows - sm))
              {
                in++;
              }
              */
             
              if(uvd.x == -1 || uvd.y == -1)
              {
                sum_e = 100000000;
                break;
              }
              
              double d = sqrt( pow(uvd.x - uvm.x,2) + pow(uvd.y - uvm.y,2)  );
              sum_e = sum_e + d;
              n_in++;
            }    

      }
      
      double mean_reprojection_error  = sum_e/n_in;
      cout << "Potential Loop: Mean reprojection error: " << mean_reprojection_error  << endl;
      
      double d_xy = sqrt( pow(tvec_u.at<double>(0) - tvec.at<double>(0),2) +  pow(tvec_u.at<double>(1) - tvec.at<double>(1),2) );

      if (mean_reprojection_error < PAR.CL_min_mean_reprojection_error && d_xy < PAR.CL_xy_update_max_delta)
      { 
        // The pose computed by the PnP technique es admited.      
        // cout << "Close loop: " << endl;
        // cout << "Initial pose: " << tvec_u.t() << endl;
        // cout << "PnP pose:     " << tvec.t() << endl;
        //cout << "intial rot: " << rvec_u << endl; 
        //cout << "PnP rot: " << rvec << endl;
        pos[0] = tvec.at<double>(0);
        pos[1] = tvec.at<double>(1);
        pos[2] = tvec.at<double>(2);
        return true;           
      }
      else
      {
         return false;
      }
   
  }
  else
  {
    return false;
  }

   
   
  

}