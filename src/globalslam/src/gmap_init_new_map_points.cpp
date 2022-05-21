#define CERES_FOUND true



#include "gmap.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/anms/anms.h"

#ifndef NOTHAVE_SFM
     #include <opencv2/sfm.hpp>
#endif

#include <cmath>
#include <numbers>
//#include <opencv2/sfm/triangulation.hpp>
//#include "opencv2/sfm/triangulation.hpp"

void matchFeatures2b(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches );

// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

//--------------------------------------------------------------------------
// 
arma::vec::fixed<3> GMAP::Triangulate_sigle_3d_point(cv::Point2f &uv1d,cv::Point2f &uv2d,arma::mat::fixed<3,3> &Rn2c_1,arma::vec::fixed<3> &t_c2n_1,arma::mat::fixed<3,3> &Rn2c_2,arma::vec::fixed<3> &t_c2n_2 )
{   
    arma::mat::fixed<3,2> dhc_duvd;  
    arma::vec::fixed<3> hc_1 = Inverse_projection_model(uv1d,1,false, cam_parameters,dhc_duvd); // compute inverse projection model
    arma::vec::fixed<3> hc_2 = Inverse_projection_model(uv2d,1,false, cam_parameters,dhc_duvd); // compute inverse projection model
    
    arma::vec::fixed<3> hn_1 = Rn2c_1.t()*hc_1;
    arma::vec::fixed<3> hn_2 = Rn2c_2.t()*hc_2;
    
    arma::vec::fixed<3> el2 = t_c2n_1 - t_c2n_2;
    arma::vec::fixed<3> el1 = t_c2n_2 - t_c2n_1;

    float hn_1_norm = arma::norm(hn_1);
    float hn_2_norm = arma::norm(hn_2);
    float el_norm = arma::norm(el2);

    hn_1 = hn_1/hn_1_norm; // normalized vector (in the nav frame) pointing in the direction of the feature
    hn_2 = hn_2/hn_2_norm; // normalized vector (in the nav frame) pointing in the direction of the feature

    float gama =  acos( arma::as_scalar(hn_1.t()*(el1)) /el_norm   ); 
    float beta =  acos( arma::as_scalar(hn_2.t()*(el2)) /el_norm   ); 
    
    float alfa = 3.141592653589793 - (beta + gama);
    
    float d_i = (el_norm*sin(gama))/sin(alfa);
    
    arma::vec::fixed<3> pt; 
    if (d_i > PAR.Init_min_depth_to_consider)
    { 
       pt = t_c2n_2 + d_i*hn_2;
    }
    else
    {
       pt = {0,0,0};
    }    
    return pt;

}


//--------------------------------------------------------------------------
// Function for computing new 3d points from triangulating from two keyframes
// Rodrigo M. 2022
std::vector<int64> GMAP::Init_new_map_points()
{   
    std::vector<int64> idx_new_points;

    static int total_n_init_points = 0;

    int idx_current_KF = Gmap.KeyFDATA.size()-1;
    int idx_previous_KF = Gmap.KeyFDATA.size()-2;

    vector<cv::KeyPoint>  keyPoints_current,keyPoints_previous;
    cv::Mat Descriptors_previous,Descriptors_current;
    
    if(PAR.Init_use_anms_for_select_strong_points == true)
    {   
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(PAR.Init_number_candidate_points_per_kf*4, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
        //----- Select strongest and spatially distributed keypoints 
            vector<float> responseVector;
            for (unsigned int i =0 ; i < Gmap.KeyFDATA[idx_current_KF].keyPoints_all.size(); i++) responseVector.push_back(Gmap.KeyFDATA[idx_current_KF].keyPoints_all[i].response);
            vector<int> Indx(responseVector.size()); 
            std::iota (std::begin(Indx), std::end(Indx), 0);
            cv::sortIdx(responseVector, Indx, cv::SORT_DESCENDING);
            vector<cv::KeyPoint> keyPointsSorted;
            for (unsigned int i = 0; i < Gmap.KeyFDATA[idx_current_KF].keyPoints_all.size(); i++) keyPointsSorted.push_back(Gmap.KeyFDATA[idx_current_KF].keyPoints_all[Indx[i]]);
            int numRetPoints = PAR.Init_number_candidate_points_per_kf; //choose exact number of return points    
            float tolerance = 0.3; // tolerance of the number of return points
            keyPoints_current = Ssc(keyPointsSorted,numRetPoints,tolerance,Gmap.KeyFDATA[idx_current_KF].frame.cols,Gmap.KeyFDATA[idx_current_KF].frame.rows);
        //----------------------------------- 
        detector->compute(Gmap.KeyFDATA[idx_current_KF].frame, keyPoints_current, Descriptors_current); // compute descriptors for Keypoints    
        detector->clear();         
        
        //----- Select strongest and spatially distributed keypoints 
            vector<float> responseVector2;
            for (unsigned int i =0 ; i<Gmap.KeyFDATA[idx_previous_KF].keyPoints_all.size(); i++) responseVector2.push_back(Gmap.KeyFDATA[idx_previous_KF].keyPoints_all[i].response);
            vector<int> Indx2(responseVector2.size()); 
            std::iota (std::begin(Indx2), std::end(Indx2), 0);
            cv::sortIdx(responseVector2, Indx2, cv::SORT_DESCENDING);
            vector<cv::KeyPoint> keyPointsSorted2;
            for (unsigned int i = 0; i < Gmap.KeyFDATA[idx_previous_KF].keyPoints_all.size(); i++) keyPointsSorted2.push_back(Gmap.KeyFDATA[idx_previous_KF].keyPoints_all[Indx2[i]]);
            int numRetPoints2 = PAR.Init_number_candidate_points_per_kf; //choose exact number of return points    
            float tolerance2 = 0.3; // tolerance of the number of return points
            keyPoints_previous = Ssc(keyPointsSorted2,numRetPoints2,tolerance2,Gmap.KeyFDATA[idx_previous_KF].frame.cols,Gmap.KeyFDATA[idx_previous_KF].frame.rows);
        //----------------------------------- 
        detector->compute(Gmap.KeyFDATA[idx_previous_KF].frame, keyPoints_previous, Descriptors_previous); // compute descriptors for Keypoints  

       
    }
    else
    {   
        keyPoints_current  = Gmap.KeyFDATA[idx_current_KF].keyPoints_all;;
        Descriptors_current = Gmap.KeyFDATA[idx_current_KF].Descriptors_all;;
        
        keyPoints_previous = Gmap.KeyFDATA[idx_previous_KF].keyPoints_all;
        Descriptors_previous = Gmap.KeyFDATA[idx_previous_KF].Descriptors_all;
       
    }

    std::vector<cv::DMatch> matches;
    // matchFeatures2b( query_des , tarjet_des, matches)   
    matchFeatures2b(Descriptors_previous, Descriptors_current, matches);  

    // -----------  Use RANSAC (Find Homograpy) to discard outliers
    std::vector<cv::Point2f> Points_m2;
    std::vector<cv::Point2f> Points_m1;
    for(unsigned int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
        {  
          int Descriptor_idx_m2 = matches[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int Descriptor_idx_m1 = matches[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f point_idx_m2 = keyPoints_previous[Descriptor_idx_m2].pt;
          cv::Point2f point_idx_m1 = keyPoints_current[Descriptor_idx_m1].pt;
          Points_m2.push_back(point_idx_m2);
          Points_m1.push_back(point_idx_m1); 
        }      
    cv::Mat status;
    //cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);
    cv::Mat H = cv::findFundamentalMat(Points_m2, Points_m1, cv::RANSAC, 3, 0.99, status );
    // obtain inliers
        std::vector<cv::DMatch> inliers;   // Store correct matches in "inliers"
        for(size_t i = 0; i < Points_m2.size(); i++) 
            {
                if(status.at<char>(i) != 0) 
                {
                    inliers.push_back(matches[i]);
                }
            }
    // ----------------------------------------------------------------------------------------
    //cout << "inliers: " << inliers.size() << endl;
    // ------------ In order to no over saturate the global map, remove matches that are to close to previous matched map points
    std::vector<cv::DMatch> Matches_final; 
    for(unsigned int i =0 ; i< inliers.size();i++ ) // for each inlier, 
       {         
         int idx_descriptor_current = inliers[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
         cv::Point2f pt_current = keyPoints_current[idx_descriptor_current].pt; // get image coordinates of i-inlier     

            bool consider_inlier = true;
            // check for every of the already matched points
            for(int j = 0; j < Gmap.KeyFDATA[idx_current_KF].Idx_Matched_points.size(); j++)
            {
               //int64 idx_already_matche_pt = Gmap.KeyFDATA[idx_current_KF].Idx_Matched_points[j];
               cv::Point2f pt_mapped = Gmap.KeyFDATA[idx_current_KF] .UV_Matched_points[j];

               double dis = sqrt( pow(pt_current.x - pt_mapped.x,2) + pow(pt_current.y - pt_mapped.y,2) );

               double min_dis = (double)PAR.Init_min_distance_to_previous_mapped_features;
               if(dis < min_dis)
                {
                   consider_inlier = false;
                   break;
                }
            }

            if(consider_inlier == true)
            {
                Matches_final.push_back(inliers[i]);
            }

       } 
    //----------------------------------------------------------------------------------------       
    // cout << "Matches_final: " << Matches_final.size() << endl;
    //-----------------------------------------------------------------------
    //  previous view: Rotation matrix and translation vector    
   arma::mat::fixed<3,3> Rn2c_2 = Gmap.KeyFDATA[idx_previous_KF].Rn2c;
   arma::vec::fixed<3> t_c2n_2 = Gmap.KeyFDATA[idx_previous_KF].t_c2n;
    arma::mat::fixed<3,3> Rn2c_1 = Gmap.KeyFDATA[idx_current_KF].Rn2c;
   arma::vec::fixed<3> t_c2n_1 = Gmap.KeyFDATA[idx_current_KF].t_c2n;  
    //---------------------------------------------------------------------- 
   
    #ifndef NOTHAVE_SFM
            // Triangulation by using OpenCV SFM module ----------------------------------------------------------

             
            cv::Mat Points_1(2,Matches_final.size(),CV_32FC1);
            cv::Mat Points_2(2,Matches_final.size(),CV_32FC1);
            //------Undistort points----------------------
            for(unsigned int i =0 ; i< Matches_final.size();i++ ) // for each match, update Feature table
                {
                    int idx_descriptor_previous = Matches_final[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
                    int idx_descriptor_current = Matches_final[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
                    cv::Point2f point_idx_m2 = keyPoints_previous[idx_descriptor_previous].pt;
                    cv::Point2f point_idx_m1 = keyPoints_current[idx_descriptor_current].pt;        

                    cv::Point2f uv1_u,uv2_u;
                    int ditortion_model = 1;
                    uv1_u = Undistort_a_point( point_idx_m1,cam_parameters,ditortion_model );  // undistort point
                    uv2_u = Undistort_a_point( point_idx_m2,cam_parameters,ditortion_model );  // undistort point                 
                                
                    Points_1.at<float>(0,i) = uv1_u.x;
                    Points_1.at<float>(1,i) = uv1_u.y;
                    Points_2.at<float>(0,i) = uv2_u.x;
                    Points_2.at<float>(1,i) = uv2_u.y;

                } 
            //----------------------------------------------------------------------
            //  Compute Projection matrices
            double fc1 = cam_parameters.fc[0];
            double fc2 = cam_parameters.fc[1];
            double cc1 = cam_parameters.cc[0];
            double cc2 = cam_parameters.cc[1];
            double alpha_c = cam_parameters.alpha_c;
            
            arma::mat::fixed<3,3>  KK = {{fc1, alpha_c*fc2, cc1},
                                        { 0 ,    fc2  ,    cc2},
                                        { 0 ,      0  ,    1  }};
            
            //arma::vec::fixed<3> t_c2n_2 = {0,0,0};

            // The reference (fixed) frame is the first (old) frame [Gmap.KeyFDATA.size()-2]
            arma::mat::fixed<3,4> RT2; //  RT2 =  [R|t] = [I|0]    
            RT2(arma::span(0,2),arma::span(0,2)) = arma::eye(3,3);
            RT2.col(3) = {0,0,0};  
            arma::mat::fixed<3,4> Pm_2a = KK*RT2;    // Kf_2 Projection matrix (armadillo format)
            arma::mat Pm_2a_t = Pm_2a.t(); // for converting to OpenCV mat    
            cv::Mat Pm2 = to_cvmat(Pm_2a_t); // Kf_2 Projection matrix (OpenCv format)
            cv::Mat PMat_2(3,4,CV_32FC1);   
            PMat_2 = Pm2; // Projection matrix reference frame (float) PMat_2 = K*[R|t] 
            
            //---Current camera:   Gmap.KeyFDATA[Gmap.KeyFDATA.size()-1]
            // Compute the projection matrix from the relative rotation and translation of the current kf with respect to the previous kf 
            // Opencv uses the convention that points (instead of cameras) are traslated from one frame to another frame
            arma::mat::fixed<3,4> RT1;
            arma::mat::fixed<3,3> R = Rn2c_1*Rn2c_2.t(); // compute the relative rotation from absolute (navigation) rotation matrix
            arma::vec::fixed<3> t = Rn2c_2*(t_c2n_2 - t_c2n_1); // compute the relative camera translation, using the opencv convention
            RT1(arma::span(0,2),arma::span(0,2)) = R;
            RT1.col(3) = t;
            arma::mat::fixed<3,4> Pm_1a = KK*RT1;    // Kf_1 Projection matrix (armadillo format)
            arma::mat Pm_1a_t = Pm_1a.t(); // for converting to OpenCV mat    
            cv::Mat Pm1 = to_cvmat(Pm_1a_t); // Kf_1 Projection matrix (OpenCv format)
            cv::Mat PMat_1(3,4,CV_32FC1);    
            PMat_1 = Pm1;   
                      
            
            // Triangulation using the SFM module from OpenCv
            std::vector<cv::Mat> points2d;
            points2d.push_back(Points_2);
            points2d.push_back(Points_1);

            //cout << points2d[0] << endl;
            //cout << points2d[1] << endl;
            
            std::vector<cv::Mat> projection_matrices;
            cv::Mat ProMat_1(3,4,CV_32F);
            cv::Mat ProMat_2(3,4,CV_32F);  
            ProMat_1 = Pm1;
            ProMat_2 = Pm2;
            
            projection_matrices.push_back(Pm2);
            projection_matrices.push_back(Pm1);    
            
            cv::Mat points3d;    
            cv::sfm::triangulatePoints(points2d,projection_matrices,points3d );    
        
            //cout << "SFM triangulate: " << points3d.size() <<  endl;
            arma::mat::fixed<3,3> Rc2n_2 = Rn2c_2.t(); 


            for(int i = 0; i < points3d.cols; i++ )
                {   
                    int idx_descriptor_previous = Matches_final[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
                    int idx_descriptor_current = Matches_final[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]

                    arma::vec::fixed<3> pt_c;
                    pt_c[0] =  points3d.at<double>(0,i);
                    pt_c[1] =  points3d.at<double>(1,i); 
                    pt_c[2] =  points3d.at<double>(2,i);
                    
                    // convert from the camera_2 Gmap.KeyFDATA[Gmap.KeyFDATA.size()-2] frame to the navigation frame
                    arma::vec::fixed<3> pt_n;
                    pt_n = Rc2n_2*pt_c + t_c2n_2;
                   // cout << pt_n.t() << endl;
                    //cout << "pt: " << x << " " << y << " " << z << endl;

                    if (pt_c[2] > PAR.Init_min_depth_to_consider )
                    { 
                        G_ANCHOR Anchor;
                        Anchor.feat_type = "g_anchor";
                        Anchor.AnchorState = pt_n;
                        Anchor.Descriptor = Descriptors_current.row(idx_descriptor_current);
                        Anchor.Keypoint = keyPoints_current[idx_descriptor_current];
                        Anchor.init_KF = idx_current_KF;
                        Anchor.n_kf_matched = 2;
                        Anchor.n_tries_matchs = 0;                           
                        Gmap.AnchorsDATA.push_back(Anchor);
                        //cout << "x"; 
                        int64 idx_anchor = Gmap.AnchorsDATA.size()-1;

                        Gmap.KeyFDATA[idx_current_KF].Idx_Matched_points.push_back(idx_anchor);
                        Gmap.KeyFDATA[idx_current_KF].UV_Matched_points.push_back(keyPoints_current[idx_descriptor_current].pt);

                        Gmap.KeyFDATA[idx_previous_KF].Idx_Matched_points.push_back(idx_anchor);
                        Gmap.KeyFDATA[idx_previous_KF].UV_Matched_points.push_back(keyPoints_previous[idx_descriptor_previous].pt);    
                    
                        idx_new_points.push_back(idx_anchor);
                    }
                } 
    #else  // if sfm is not found

      //--------------------------------------------------------  
      // Triangulation by Simple triangulation method -------------------------

       //cout << "Simple triangulate:" << endl; 
        for(unsigned int i =0 ; i< Matches_final.size();i++ ) // for each match, update Feature table
        {
          int idx_descriptor_previous = Matches_final[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int idx_descriptor_current = Matches_final[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f uv2d = keyPoints_previous[idx_descriptor_previous].pt;
          cv::Point2f uv1d = keyPoints_current[idx_descriptor_current].pt;   
        
          arma::vec::fixed<3> pt_n = Triangulate_sigle_3d_point(uv1d,uv2d,Rn2c_1,t_c2n_1,Rn2c_2,t_c2n_2 );         
          //cout << pt_n.t() << endl;
          
          if(!(pt_n[0] == 0 && pt_n[1] == 0 && pt_n[2] == 0) )
          {
            G_ANCHOR Anchor;
            Anchor.feat_type = "g_anchor";
            Anchor.AnchorState = pt_n;
            Anchor.Descriptor = Descriptors_current.row(idx_descriptor_current);
            Anchor.Keypoint = keyPoints_current[idx_descriptor_current];
            Anchor.init_KF = idx_current_KF;
            Anchor.n_kf_matched = 2;  

            Gmap.AnchorsDATA.push_back(Anchor);

            int64 idx_anchor = Gmap.AnchorsDATA.size()-1;

                        Gmap.KeyFDATA[idx_current_KF].Idx_Matched_points.push_back(idx_anchor);
                        Gmap.KeyFDATA[idx_current_KF].UV_Matched_points.push_back(keyPoints_current[idx_descriptor_current].pt);

                        Gmap.KeyFDATA[idx_previous_KF].Idx_Matched_points.push_back(idx_anchor);
                        Gmap.KeyFDATA[idx_previous_KF].UV_Matched_points.push_back(keyPoints_previous[idx_descriptor_previous].pt);    
            
            idx_new_points.push_back(idx_anchor);
          }  
        }
     
    #endif

    total_n_init_points = total_n_init_points + Matches_final.size();    

   int q = 10;

return idx_new_points;
}




//----------------------------------------------------------------------------------------------------------------
#define RATIO    0.75
//#define RATIO    0.85
void matchFeatures2b(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
{
    std::vector<std::vector<cv::DMatch>> matches;
    
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    
    matcher.knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {   
        if (!matches[i].empty())
            if (matches[i][0].distance < matches[i][1].distance * RATIO)
                goodMatches.push_back(matches[i][0]);
    }
}