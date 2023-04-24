#include "cloop.hpp"

void matchFeatures4(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches); 


//-------------------------------------------------------------------------------------------------------
void CLOOP::Match_list_of_points_into_KF(std::vector<int64> &idx_points, int idx_KF,std::vector<int64> &idx_matched_points)
{
  vector<cv::KeyPoint> keyPoints_A;
  cv::Mat Descriptors_A;
  std::vector<cv::Point2d> Predicted_projections; // vector of predicted projections into the KF
  
  arma::mat::fixed<3,3> Rn2c = Gmap.KeyFDATA[idx_KF].Rn2c;
  arma::vec::fixed<3> t_c2n = Gmap.KeyFDATA[idx_KF].t_c2n; 
  
  std::vector<int64> Idx_candidate_A;
  for( int i = 0; i < idx_points.size(); i++)
  { 
    int64 idx_pt = idx_points[i];                   
    arma::vec::fixed<3> pxyz = Gmap.AnchorsDATA[idx_pt].AnchorState;
    arma::vec::fixed<3> pc = Rn2c*(pxyz - t_c2n); // feature point expressed in the camera frame
    arma::mat::fixed<2,3> duv_dPc;
    cv::Point2d uvd = Projection_model(pc,0,false,cam_parameters,duv_dPc ); 

    int sm =  PAR.VM_max_inov_error;        
    if((uvd.x > -sm)&&(uvd.x < PAR.Mono_cam_img_cols + sm)&&(uvd.y > -sm)&&(uvd.y < PAR.Mono_cam_img_rows + sm))
        {
          Idx_candidate_A.push_back(idx_pt);
          Descriptors_A.push_back(Gmap.AnchorsDATA[idx_pt].Descriptor);
          keyPoints_A.push_back(Gmap.AnchorsDATA[idx_pt].Keypoint);   
          Predicted_projections.push_back(uvd);
        }       
  }
  
  if (Predicted_projections.size() > 0)
    {
        // try to match
        std::vector<cv::DMatch> matches;
        // Descriptors_A = query,  // KF_descriptors = target
        matchFeatures4(Descriptors_A,Gmap.KeyFDATA[idx_KF].Descriptors_all, matches);

        for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table          
          { 
            int Descriptor_idx_A =  matches[k].queryIdx; // get index of the descriptor in Descriptors_A    
            int Descriptor_idx_KF = matches[k].trainIdx; // get index of the descriptor in KeyFframe  
            
            cv::Point2d pt_m = Gmap.KeyFDATA[idx_KF].keyPoints_all[Descriptor_idx_KF].pt;  // coordinates of the matched point
            cv::Point2d pt_p = Predicted_projections[Descriptor_idx_A]; // coordinates of the predicted point

            double dis = sqrt( pow(pt_p.x - pt_m .x,2) + pow(pt_p.y - pt_m .y,2)  );

            // check innovation error  (norm(predicted-matched))
            double max_inov_error = (double)PAR.VM_max_inov_error;
            if(dis < max_inov_error)
              {
                int64 Idx_matched_A = Idx_candidate_A[Descriptor_idx_A]; // get index of the matched map point (anchor)
                
                idx_matched_points.push_back(Idx_matched_A);
                Gmap.KeyFDATA[idx_KF].Idx_Matched_points.push_back(Idx_matched_A); // Add the index of the matched map point to the list of Kf matched points
                Gmap.KeyFDATA[idx_KF].UV_Matched_points.push_back(pt_m);
                
                Gmap.AnchorsDATA[Idx_matched_A].n_kf_matched++; 
              } 
        
          }
    } // if (Predicted_projections.size() > 0)     




}




//---------------------------------------------------------------------------------------------------------------
void CLOOP::Get_matches(KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches,std::vector<cv::Point2d> &image_points,int &idx_kf_matched)
{

  int idx_current_kf = Gmap.KeyFDATA.size()- 1;
   // cout << "r_N: " << kf_cl.r_N << endl;

  // ---- get candidate global map points to be matched into current frame (kf_cl)

  std::vector<int> idx_nvl_kf = Get_n_not_visually_linked_kf(idx_current_kf, Gmap.KeyFDATA.size()- PAR.BA_max_n_kf_optimized );
  
  
  if ( idx_nvl_kf.size() > PAR.CL_min_n_not_vl_kf)
  {
    vector<cv::KeyPoint> keyPoints_KF_cl;
    cv::Mat Descriptors_KF_cl;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(PAR.Init_number_candidate_points_per_kf*4, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
    detector->detectAndCompute(kf_cl.frame,cv::Mat(), keyPoints_KF_cl,Descriptors_KF_cl);

      for(int i = 0; i < idx_nvl_kf.size();i++)
      {      
          int idx_kf = idx_nvl_kf[i]; // index of the not visually linked kf
          cv::Mat Descriptors_C; // For storing descriptors of candidate anchors (map points)
          std::vector<int64> Idx_candidate_C;  // index candidate anchors 
          
          for(int j = 0; j < Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size(); j++)
          {        
              int64 idx_pt = Gmap.KeyFDATA[idx_kf].Idx_Matched_points[j];
              Idx_candidate_C.push_back(idx_pt);  // add index of candidate map point
              Descriptors_C.push_back(Gmap.AnchorsDATA[idx_pt].Descriptor); // add descriptor of candidate map point     
          
          }

          std::vector<cv::DMatch> matches;
          // Descriptors_C = query,  // Descriptors_KF_cl = target
          matchFeatures4(Descriptors_C,Descriptors_KF_cl, matches); 
          
          if(matches.size() > PAR.CL_min_n_matches )
          {
            for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table          
            { 
              int Descriptor_idx_np =  matches[k].queryIdx; // get index of the descriptor in Descriptors_np    
              int Descriptor_idx_KF =  matches[k].trainIdx; // get index of the descriptor in KeyFframe  

              int idx_new_pt = Idx_candidate_C[Descriptor_idx_np];

              cv::Point2d pt_m = keyPoints_KF_cl[Descriptor_idx_KF].pt;  // coordinates of the matched point

              idx_pt_matches.push_back(idx_new_pt);
              image_points.push_back(pt_m);
              
            }
            idx_kf_matched = idx_kf;
            //cout << "Potential loop detected: " << matches.size() << " matches with KF: " << idx_kf << endl;    
          break;
          }
      
      
      }    


    

  }  



}


//------------------------------------------------------------
// Try to match already mapped points into the new KeyFrame
// Rodrigo M. 2022
void CLOOP::Match_Gmap_points_into_KF(int idx_KF)
{
 

  vector<cv::KeyPoint> keyPoints_KF;
  cv::Mat Descriptors_KF;

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(PAR.Init_number_candidate_points_per_kf*4, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
  detector->detectAndCompute(Gmap.KeyFDATA[idx_KF].frame,cv::Mat(), keyPoints_KF,Descriptors_KF);

  // store all keypoints and descriptors for the current kf (this will be used later for matching)
  Gmap.KeyFDATA[idx_KF].keyPoints_all = keyPoints_KF;
  Gmap.KeyFDATA[idx_KF].Descriptors_all = Descriptors_KF;

  cv::Mat Descriptors_A; // For storing descriptors of candidate anchors (map points)
  std::vector<int64> Idx_candidate_A;  // index candidate anchors
  std::vector<cv::Point2d> Predicted_projections; // vector of predicted projections into the KF
  // search for map points that can be potentially matched into the new KF 
   for(int i = Gmap.AnchorsDATA.size()-1; i >=0 ; i--)
    { 
      int idx_kf_min_ref_for_search_matches = Gmap.KeyFDATA.size() - PAR.BA_max_n_kf_optimized;
      if(idx_kf_min_ref_for_search_matches<0)idx_kf_min_ref_for_search_matches = 0;
      // to avoid to search for matches in keyframes that will not be taken into acount for bundle adjustment   
      if(Gmap.AnchorsDATA[i].init_KF > idx_kf_min_ref_for_search_matches)
      {   
         arma::mat::fixed<3,3> Rn2c = Gmap.KeyFDATA[idx_KF].Rn2c;
         arma::vec::fixed<3> t_c2n = Gmap.KeyFDATA[idx_KF].t_c2n;                
         arma::vec::fixed<3> pxyz = Gmap.AnchorsDATA[i].AnchorState;
         arma::vec::fixed<3> pc = Rn2c*(pxyz - t_c2n); // feature point expressed in the camera frame
         arma::mat::fixed<2,3> duv_dPc;
         cv::Point2d uvd = Projection_model(pc,0,false,cam_parameters,duv_dPc );
         
         Gmap.AnchorsDATA[i].n_tries_matchs++;

         // check if the feature is predicted to appear in the image plus a margin
        int sm =  PAR.VM_max_inov_error;        
        if((uvd.x > -sm)&&(uvd.x < PAR.Mono_cam_img_cols + sm)&&(uvd.y > -sm)&&(uvd.y < PAR.Mono_cam_img_rows + sm))
        {

          Idx_candidate_A.push_back(i);  // add index of candidate map point
          Descriptors_A.push_back(Gmap.AnchorsDATA[i].Descriptor); // add descriptor of candidate map point
          Predicted_projections.push_back(uvd);
        }  
      }
    }  
    
    if (Predicted_projections.size() > 0)
    {
        // try to match
        std::vector<cv::DMatch> matches;
        // Descriptors_A = query,  // KF_descriptors = target
        matchFeatures4(Descriptors_A,Gmap.KeyFDATA[idx_KF].Descriptors_all, matches);

        for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table          
          { 
            int Descriptor_idx_A =  matches[k].queryIdx; // get index of the descriptor in Descriptors_A    
            int Descriptor_idx_KF = matches[k].trainIdx; // get index of the descriptor in KeyFframe  
            
            cv::Point2d pt_m = keyPoints_KF[Descriptor_idx_KF].pt;  // coordinates of the matched point
            cv::Point2d pt_p = Predicted_projections[Descriptor_idx_A]; // coordinates of the predicted point

            double dis = sqrt( pow(pt_p.x - pt_m .x,2) + pow(pt_p.y - pt_m .y,2)  );

            // check innovation error  (norm(predicted-matched))
            double max_inov_error = (double)PAR.VM_max_inov_error;
            if(dis < max_inov_error)
              {
                int64 Idx_matched_A = Idx_candidate_A[Descriptor_idx_A]; // get index of the matched map point (anchor)

                Gmap.KeyFDATA[idx_KF].Idx_Matched_points.push_back(Idx_matched_A); // Add the index of the matched map point to the list of Kf matched points
                Gmap.KeyFDATA[idx_KF].UV_Matched_points.push_back(pt_m);
                
                Gmap.AnchorsDATA[Idx_matched_A].n_kf_matched++; 
              } 
        
          }
    } // if (Predicted_projections.size() > 0)     

};

//----------------------------------------------------------------------------------------------------------
void CLOOP::Match_a_points_with_b_points(std::vector<int64> &idx_a_points,std::vector<int64> &idx_b_points,std::vector<int64> &idx_matched_a_pts,std::vector<int64> &idx_matched_b_pts)
{
    
    cv::Mat Descriptors_A;
    vector<cv::KeyPoint>  keyPoints_A;
    for (int i = 0; i < idx_a_points.size() ; i++)
    {
       int64 idx_pt = idx_a_points[i];
       Descriptors_A.push_back(Gmap.AnchorsDATA[idx_pt].Descriptor); 
       keyPoints_A.push_back(Gmap.AnchorsDATA[idx_pt].Keypoint);        
    }
    cv::Mat Descriptors_B;
    vector<cv::KeyPoint>  keyPoints_B;
    for (int i = 0; i < idx_b_points.size() ; i++)
    {
       int64 idx_pt = idx_b_points[i];
       Descriptors_B.push_back(Gmap.AnchorsDATA[idx_pt].Descriptor);
       keyPoints_B.push_back(Gmap.AnchorsDATA[idx_pt].Keypoint);          
    }

    std::vector<cv::DMatch> matches;
    // matchFeatures2b( query_des , tarjet_des, matches)   
    matchFeatures4(Descriptors_A, Descriptors_B, matches);  

   
    if(matches.size()>0)
    {
        // -----------  Use RANSAC (Find findFundamentalMat) to discard outliers
        std::vector<cv::Point2f> Points_m2;
        std::vector<cv::Point2f> Points_m1;
        for(unsigned int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
            {  
              int Descriptor_idx_A = matches[i].queryIdx; // get index of the descriptor in Descriptors_A
              int Descriptor_idx_B = matches[i].trainIdx; // get index of the descriptor in Descriptors_B
              cv::Point2f point_idx_m2 = keyPoints_A[Descriptor_idx_A].pt;
              cv::Point2f point_idx_m1 = keyPoints_B[Descriptor_idx_B].pt;
              Points_m2.push_back(point_idx_m2);
              Points_m1.push_back(point_idx_m1); 
            }      
        cv::Mat status;
        //cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);
        cv::Mat H = cv::findFundamentalMat(Points_m2, Points_m1, cv::RANSAC, 2, 0.99, status );
        // obtain inliers
            std::vector<cv::DMatch> inliers;   // Store correct matches in "inliers"
            for(size_t i = 0; i < Points_m2.size(); i++) 
                {
                    if(status.at<char>(i) != 0) 
                    {
                        inliers.push_back(matches[i]);
                    }
                }
        //-----------------------------------------------------------------------------------------

        for(int i = 0; i < inliers.size(); i++)
        {
          int idx_A = matches[i].queryIdx; // get index of the inlier point in idx_a_points
          int idx_B = matches[i].trainIdx; // get index of the inlier point in idx_b_points

          int64 idx_m_pt_a = idx_a_points[idx_A];
          int64 idx_m_pt_b = idx_b_points[idx_B];

          idx_matched_a_pts.push_back(idx_m_pt_a);
          idx_matched_b_pts.push_back(idx_m_pt_b);

        }        
    
    }//  if(matches.size()>0)
    int q = 10;        



}





//---------------------------------------------------------------------------------------------------------
#define RATIO    0.85
//#define RATIO    0.85
void matchFeatures4(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
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