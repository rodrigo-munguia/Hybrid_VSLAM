#ifndef LOCALSLAM_TYPES_H
#define LOCALSLAM_TYPES_H

#include <armadillo>
#include "opencv2/opencv.hpp"



using namespace std;


//------  keyframe structure
struct KEYFRAME
{
    cv::Mat frame; // image frame
    arma::vec::fixed<3> t_c2r;  // camera to robot position vector
    arma::mat::fixed<3,3> Rr2c; // robot to camera rotation matrix 
    arma::vec::fixed<3> r_N; // position of the robot, expressed in the navigation frame 
    arma::mat::fixed<3,3> Rn2r; // navigation to robot rotation matrix
    

};



#endif