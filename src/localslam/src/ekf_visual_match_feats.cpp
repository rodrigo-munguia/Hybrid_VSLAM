#include "ekf.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "ekf_Jacobians.hpp"




int EKF::Visual_match_feats(FRAME *frame)
{

    
      // Use features descriptors for matching process
     int n_matches = Visual_match_feats_by_descriptors_e(frame); 
   
    
    return n_matches;
}




#define RATIO    0.75
//#define RATIO    0.85
void matchFeatures(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
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
//--------------------------------------------------------------------------

// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

//----------------------------------------------------------------------------------------------------
// Function for matching visual features frame to frame using visual descriptors
// Attitude of the robot is expressed by euler angles
// Rodrigo M. 2021
int EKF::Visual_match_feats_by_descriptors_e(FRAME *frame)
{
        
    cv::Mat Descriptors;
    std::vector<int> FeatIndex;
    int n_p_feats;   // number of predicted features
    int n_p_anchors = 0; // bumber of predicted anchors
    int n_m_feats = 0; // number of matched features
    int n_m_anchors = 0; // number of matched anchors 
    
         
         arma::mat::fixed<3,3> Rc2r = Rr2c.t();

         double phi = x(1);
         double theta = x(2);
         double psi = x(3);
         double Ra2b[9];
         Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
         arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
         arma::mat::fixed<3,3> Rr2n = Rn2r.t();
         arma::mat::fixed<3,3> Rc2n = Rr2n*Rc2r;
         arma::mat::fixed<3,3> Rn2c = Rc2n.t();
         arma::vec::fixed<3> r_N = x.subvec(7,9);        
        

    // --- check if Features are predicted to appear in the current frame   
        for (int i= 0; i< FeatsDATA.size();i++)
        {
            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;           

            arma::vec::fixed<3> Tn2c = r_N + Rr2n*t_c2r;  // Tn2c = r^N + Rr2n*Tc2r
            arma::vec::fixed<3> pxyz = x.subvec(idx_i,idx_f);
            
            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,cam_parameters,duv_dPc );
            int sm =  PAR.Images_half_patch_size_when_initialized;

            // check if the feature is predicted to appear in the image 
            if((uvd.x > sm)&&(uvd.x < PAR.Mono_cam_img_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.Mono_cam_img_rows - sm))
            {
                FeatsDATA[i].PredictedPoint = uvd;                 
                
                Descriptors.push_back(FeatsDATA[i].Descriptor);
                
                FeatIndex.push_back(i);

                FeatsDATA[i].predicted = true; 
                FeatsDATA[i].matched = false;   // set by default 
                FeatsDATA[i].times_not_mathed++; // increment by defualt, if later teh feat is matched then this value is decremented              
                FeatsDATA[i].times_not_considered = 0; // if a feat is "seen" again reset the number of times not considered
            }
            else // the feature is predicted out of the image
            {
                FeatsDATA[i].predicted = false;
                FeatsDATA[i].times_not_considered++; // increment
                FeatsDATA[i].matched = false;
            }
        

        } // for (int i= 0; i< FeatsDATA.size();i++)
        n_p_feats = FeatIndex.size();
        
        // check if anchors are predicted to appear in the current frame
        std::vector<int> AnchorIndex;
        for(int i = 0; i< AnchorsDATA.size(); i++)
        {
                   

            arma::vec::fixed<3> Tn2c = r_N + Rr2n*t_c2r;  // Tn2c = r^N + Rr2n*Tc2r
            arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;            

            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,cam_parameters,duv_dPc );
            int sm =  PAR.Images_half_patch_size_when_initialized;

            // check if the Anchor is predicted to appear in the image 
            if((uvd.x > sm)&&(uvd.x < PAR.Mono_cam_img_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.Mono_cam_img_rows - sm))
            {
                AnchorsDATA[i].PredictedPoint = uvd;  

                Descriptors.push_back(AnchorsDATA[i].Descriptor);
                AnchorIndex.push_back(i);

                AnchorsDATA[i].predicted = true; 
                AnchorsDATA[i].matched = false;   // set by default 
                AnchorsDATA[i].times_not_mathed++; // increment by defualt, if later teh feat is matched then this value is decremented              
                AnchorsDATA[i].times_not_considered = 0;

                n_p_anchors++;                
            }    
            else // the feature is predicted out of the image
            {
                AnchorsDATA[i].predicted = false;
                AnchorsDATA[i].times_not_considered++; // increment
                AnchorsDATA[i].matched = false;
            }

        }
        //---------------------------------------------------------------------------------------------------

        // obtain candidate Keypoint to be matched
        vector<cv::KeyPoint> Candidate_keyPoints;
        cv::Mat CandidateDescriptors;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
        detector->detectAndCompute(frame->image,cv::Mat(), Candidate_keyPoints,CandidateDescriptors);
        
        // obtain matches for visual features predicted to appear in the image.
        std::vector<cv::DMatch> matches;
        matchFeatures(Descriptors, CandidateDescriptors, matches);


        // Innovation check and Update features table 
        for(int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
        {         
                 
            int Descriptor_idx = matches[i].queryIdx; // get index of the descriptor matched

            if (Descriptor_idx < n_p_feats)  // A feature has been matched
            {

                    int Feat_idx = FeatIndex[Descriptor_idx]; // get index of the feature matched
                    int CandidateDescriptor_idx = matches[i].trainIdx; // get index of the candidate keypoint matched
                    cv::Point2f measured_point = Candidate_keyPoints[CandidateDescriptor_idx].pt;
                    cv::Point2f predicted_point = FeatsDATA[Feat_idx].PredictedPoint;

                    // Simple extra check for matching
                        
                        double d = sqrt(pow(measured_point.x - predicted_point.x,2) + pow(measured_point.y - predicted_point.y,2)); 

                        if( d < PAR.Images_max_innov_pixels )
                        {    
                            FeatsDATA[Feat_idx].matched = true;
                            FeatsDATA[Feat_idx].times_not_mathed--;
                            FeatsDATA[Feat_idx].times_mathed++;             
                            FeatsDATA[Feat_idx].Keypoint = Candidate_keyPoints[CandidateDescriptor_idx]; // Update Feature Keypoint
                            FeatsDATA[Feat_idx].MatchedPoint = Candidate_keyPoints[CandidateDescriptor_idx].pt; // Update position of the feature
                            n_m_feats++;
                        }
                   

            }
            else // An anchor has been matched
            {   
                int Anchor_idx = AnchorIndex[Descriptor_idx-n_p_feats]; // get index of the feature matched
                int CandidateDescriptor_idx = matches[i].trainIdx; // get index of the candidate keypoint matched
                cv::Point2f measured_point = Candidate_keyPoints[CandidateDescriptor_idx].pt;
                cv::Point2f predicted_point = AnchorsDATA[Anchor_idx].PredictedPoint;

                double d = sqrt(pow(measured_point.x - predicted_point.x,2) + pow(measured_point.y - predicted_point.y,2)); 

                    if( d < PAR.Images_max_innov_pixels ) // Simple extra check for matching
                    { 
                        AnchorsDATA[Anchor_idx].matched = true;
                        AnchorsDATA[Anchor_idx].times_not_mathed--;
                        AnchorsDATA[Anchor_idx].times_mathed++;             
                        AnchorsDATA[Anchor_idx].Keypoint = Candidate_keyPoints[CandidateDescriptor_idx]; // Update Feature Keypoint
                        AnchorsDATA[Anchor_idx].MatchedPoint = Candidate_keyPoints[CandidateDescriptor_idx].pt; // Update position of the feature  
                        n_m_anchors++;
                        
                    }
              
            }
            
            
           
        }
        //----------------------------------------------------------------------
        
        //cout << "n pred feats: " << n_p_feats << " n matched feats: " << n_m_feats << "  n pred anchors: " << n_p_anchors << " n matched anchors: " << n_m_anchors << endl;
       int n_m_points = n_m_feats + n_m_anchors;
       
       // Select KeyFrames
       Visual_match_SelectKeyFrames(frame,n_m_points,t_c2r, Rr2c, r_N, Rn2r);
      
               

       return n_m_points; // return the sum of the number of matched feats/anchors
       
}

//----------------------------------------------------------------------------------------------------
// Function for selecting KeyFrames
// Rodrigo M. 2021

void EKF::Visual_match_SelectKeyFrames(FRAME *frame, int n_m_points,arma::vec::fixed<3> t_c2r, arma::mat::fixed<3,3> Rr2c, arma::vec::fixed<3> r_N, arma::mat::fixed<3,3> Rn2r)
{   
    static bool init_kf = false;

    static int count_f=0;
    static arma::vec::fixed<3> last_robot_pos = {0,0,0};
    count_f++;
    
    // compute average depth of matched features/anchors ---------------
    double s_d = 0;    
    for(int i=0;i<FeatsDATA.size();i++)
        {  
           if(FeatsDATA[i].matched == true)
           { 
                double d =  arma::norm(x.subvec(FeatsDATA[i].idx_i_state,FeatsDATA[i].idx_f_state) - FeatsDATA[i].CameraState  ) ;
                s_d = s_d + d;
           } 
        }
    for(int i=0;i<AnchorsDATA.size();i++)
        {  
           if(AnchorsDATA[i].matched == true)
           {  
                double d =  arma::norm(AnchorsDATA[i].AnchorState - AnchorsDATA[i].CameraState  ) ;
                s_d = s_d + d;
           }      
        }
    double mean_feat_depth = s_d/(double)n_m_points; // average feature depth
    //------------------------------------------------------- 
    double dist_travel_by_robot = arma::norm(last_robot_pos - x.subvec(7,9) ) ; // distance from last KeyFrame
    
    double ratio_dist_pd = dist_travel_by_robot/mean_feat_depth;
    
    
    


    if (init_kf == true &&(count_f > PAR.Select_KeyFrames_min_n_frames_between_kf)&&(n_m_points > PAR.Select_KeyFrames_min_n_matches)&&(ratio_dist_pd > PAR.Select_KeyFrames_min_ratio_distance_depth))
        { 

           KF_selected.frame = frame->image;
           KF_selected.r_N = r_N;
           KF_selected.Rn2r = Rn2r;
           KF_selected.t_c2r = t_c2r;
           KF_selected.Rr2c = Rr2c;

           /*           
           cout << "New Keyframe selected " << endl;
           cout << "r_N: " << r_N << endl;
           cout << "Rn2r: " << Rn2r << endl;
           cout << "t_c2r: " << t_c2r << endl;
           cout << "Rr2c: " << Rr2c << endl;
           */

           count_f = 0;
           last_robot_pos = x.subvec(7,9);
           NewKF_available = true; 

        }

    if(init_kf ==false && (n_m_points > PAR.Select_KeyFrames_min_n_matches))
    {
           KF_selected.frame = frame->image;
           KF_selected.r_N = r_N;
           KF_selected.Rn2r = Rn2r;
           KF_selected.t_c2r = t_c2r;
           KF_selected.Rr2c = Rr2c;
           count_f = 0;
           last_robot_pos = x.subvec(7,9);
           NewKF_available = true;

           init_kf = true; 

    }           
   
   

}