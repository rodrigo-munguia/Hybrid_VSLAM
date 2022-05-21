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
    arma::mat::fixed<3,3> Rn2c;
    arma::vec::fixed<3> t_c2n;
    vector<cv::KeyPoint> keyPoints_all; // Keypoints associated with the KeyFrame
    cv::Mat Descriptors_all;   //Descriptors of the Keypoints
    vector<int64> Idx_Matched_points;  // Index of anchors that are mathed in the i-keyframe
    vector<cv::Point2d> UV_Matched_points; // Pixel coordinates of the ancrhos matched in the i-keyframe
     

};


struct KeyFramesData
 {
     int KeyF_idx;  // index of Keyframes associated with the Mapfeature
     bool matched;
     cv::Point2f PredictedPoint; // Predicted pixel position of the feature
     cv::Point2f MatchedPoint;  // Matched (measured) position of the feature
 };

struct G_ANCHOR
{       
    string feat_type; // Feature tupe: euclidean (XYZ): ID, etc
    arma::vec::fixed<3> AnchorState; //
    cv::Mat Descriptor;  //  Keypoint descriptor
    std::vector<KeyFramesData> iKeyFrame; // vector for storing data for the i-KeyFrame
    cv::KeyPoint Keypoint; // current feature Keypoint (include actual matched pixel position of the feature)
    int init_KF; //  index of the KeyFrame  where the Feature-anchor was initialized
    int n_kf_matched; // number of kf where the map point has been matched
    int n_tries_matchs; // number of times that the point has been considered to be matched
};


// Global map structure
struct GLOBAL_MAP
{
    std::vector<KEYFRAME> KeyFDATA;  // Structure for storing KeyFrames
    std::vector<G_ANCHOR> AnchorsDATA; // Structure for storing map points (anchors)
    arma::mat Vgraph; // visibility graph 
    //int idx_Fixed_kf_ref_BA; // index of the i-(oldest) keyFrame that will be considered "fixed" for BA
    int idx_ref_pose_slam;
    
    
};




#endif