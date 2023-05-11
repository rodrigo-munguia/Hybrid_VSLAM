#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include "opencv2/opencv.hpp"
#include <armadillo>



using namespace std;


//------  Map feature structure


struct FEAT
{
    string feat_type; // Feature tupe: euclidean (XYZ): ID, etc
    int idx_i_state;  // initial index of the feature in the system state
    int idx_f_state;  // final index of the feature in the system state
    cv::KeyPoint Initial_KeyPoint; //  feature Keypoint when it was first detected
    cv::KeyPoint Keypoint; // current feature Keypoint (include actual matched pixel position of the feature)
    cv::Mat Descriptor;  //  Keypoint descriptor
    int times_mathed;
    int times_not_mathed;
    int times_not_considered;
    bool predicted; // the image is predicted to "appear" in the current image
    bool matched; // the feature was matched in the current frame ? 
    arma::mat::fixed<2,2> Si;  // Innovation matrix at step i
    arma::mat::fixed<2,13> duv_dx; // Jacobian respect to camera/robot state
    arma::mat::fixed<2,3> duv_dy; // Jacobian respect to feature
    cv::Point2f PredictedPoint; // Predicted pixel position of the feature
    cv::Point2f MatchedPoint; // Actual pixel position matched fo the feature
    arma::vec::fixed<3> CameraState; // State of the camera at the moment that the feature was initialized
    arma::vec::fixed<3> AnchorState; //                       
    cv::Mat Patch_when_initialized;
    cv::Mat Patch_when_matching;
    int id_anchor; // Unique anchor identifier 

};

/*
struct CAM
{
  double *distortions;
  double cc[2];
  double fc[2];
  double alpha_c;  
};
*/

struct STORE
{
    // for plotting  

    int n_init_feats;
    int n_init_anchors;
    int n_delete_feats;
    int n_delete_anchors;
    
    std::pair<std::vector<double>,std::vector<double>> n_feats_per_frame;
    std::pair<std::vector<double>,std::vector<double>> n_anchors_per_frame;
    std::pair<std::vector<double>,std::vector<double>> time_per_frame;    
    
    double total_exec_time;  //  total time running the algorithm
    double total_comp_time;  // total computation time

};    





#endif