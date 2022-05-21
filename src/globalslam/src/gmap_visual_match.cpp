#include "gmap.hpp"



void matchFeatures3(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches); 

//------------------------------------------------------------
// Try to match already mapped points into the new KeyFrame
// Rodrigo M. 2022
void GMAP::Match_Gmap_points_into_new_KF()
{
  int idx_KF = Gmap.KeyFDATA.size()-1;

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
  int idx_kf_min_ref_for_search_matches = Gmap.KeyFDATA.size() - PAR.BA_max_n_previous_kf_for_search_matches;
      if(idx_kf_min_ref_for_search_matches<0)idx_kf_min_ref_for_search_matches = 0;
   

   for(int i = Gmap.AnchorsDATA.size()-1; i >=0 ; i--)
    { 
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
        matchFeatures3(Descriptors_A,Gmap.KeyFDATA[idx_KF].Descriptors_all, matches);

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


//------------------------------------------------------------
// Try to match new initialized points into previous KeyFrames
// Rodrigo M. 2022
void GMAP::Match_New_points_into_previous_KF(std::vector<int64> &idx_new_points, std::vector<int> &idx_kf_m, std::vector<int64> &idx_pt_m )
{
  cv::Mat Descriptors_np; // For storing descriptors of new points (map points)

  // Add descriptors of new points
  for(int i = 0; i < idx_new_points.size(); i++)
  {    
    Descriptors_np.push_back( Gmap.AnchorsDATA[idx_new_points[i]].Descriptor);

    Gmap.AnchorsDATA[idx_new_points[i]].n_tries_matchs++;
  }

  if (Gmap.KeyFDATA.size() > 2)
  {   
      int idx_current_KF = Gmap.KeyFDATA.size()-1;
      int idx_previous_KF = Gmap.KeyFDATA.size()-2;
      int idx_old_KF = idx_previous_KF - PAR.BA_max_n_kf_optimized;
      if(idx_old_KF < 0)idx_old_KF = 0;
      //cout << Gmap.Vgraph << endl;

      std::vector<int> idx_linked_KF;

        /*
        std::vector<int> idx_linked_KF_t; 
        idx_linked_KF_t = Get_idx_visually_linked_to_KF(idx_previous_KF);         
        for(int i = 0; i < idx_linked_KF_t.size(); i++)
        {
          int idx_kf = idx_linked_KF_t[i];
          if((idx_kf != idx_current_KF) && ( idx_kf != idx_previous_KF)  )
          {
            idx_linked_KF.push_back(idx_kf);
          }
        }
        */
      int idx_kf_min_ref_for_search_matches = Gmap.KeyFDATA.size() - PAR.BA_max_n_previous_kf_for_search_matches;
      if(idx_kf_min_ref_for_search_matches<0)idx_kf_min_ref_for_search_matches = 0;
    
      for(int i = 0; i < idx_previous_KF; i++)
      {
        if (i > idx_kf_min_ref_for_search_matches)
        {
          idx_linked_KF.push_back(i);
        }
      }


      for (int i = 0; i < idx_linked_KF.size() ; i++)
      {       
              int idx_kf_target = idx_linked_KF[i];
              // to avoid to search for matches in keyframes that will not be taken into acount for bundle adjustment  
              if(idx_kf_target >= idx_kf_min_ref_for_search_matches)   
              {
                // try to match
                std::vector<cv::DMatch> matches;
                // Descriptors_A = query,  // KF_descriptors = target
                matchFeatures3(Descriptors_np,Gmap.KeyFDATA[idx_kf_target].Descriptors_all, matches); 

                // check matches
                 for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table          
                 { 
                    int Descriptor_idx_np =  matches[k].queryIdx; // get index of the descriptor in Descriptors_np    
                    int Descriptor_idx_KF =  matches[k].trainIdx; // get index of the descriptor in KeyFframe  

                    int idx_new_pt = idx_new_points[Descriptor_idx_np];
                    
                    
                    cv::Point2d pt_m = Gmap.KeyFDATA[idx_kf_target].keyPoints_all[Descriptor_idx_KF].pt;  // coordinates of the matched point 

                    arma::mat::fixed<3,3> Rn2c = Gmap.KeyFDATA[idx_kf_target].Rn2c;
                    arma::vec::fixed<3> t_c2n = Gmap.KeyFDATA[idx_kf_target].t_c2n;                
                    arma::vec::fixed<3> pxyz = Gmap.AnchorsDATA[idx_new_pt].AnchorState;
                    arma::vec::fixed<3> pc = Rn2c*(pxyz - t_c2n); // feature point expressed in the camera frame
                    arma::mat::fixed<2,3> duv_dPc;
                    cv::Point2d pt_p = Projection_model(pc,0,false,cam_parameters,duv_dPc ); // get coordinates of predicted point

                    double dis = sqrt( pow(pt_p.x - pt_m .x,2) + pow(pt_p.y - pt_m .y,2)  );

                    // check innovation error  (norm(predicted-matched))
                    double max_inov_error = (double)PAR.VM_max_inov_error;
                    if(dis < max_inov_error)
                    {
                      Gmap.KeyFDATA[idx_kf_target].Idx_Matched_points.push_back(idx_new_pt); // Add the index of the matched map point to the list of Kf matched points
                      Gmap.KeyFDATA[idx_kf_target].UV_Matched_points.push_back(pt_m);
                
                      Gmap.AnchorsDATA[idx_new_pt].n_kf_matched++; 
                      
                      idx_kf_m.push_back(idx_kf_target); 
                      idx_pt_m.push_back(idx_new_pt); 
                     // cout << "Map point: " << idx_new_pt << " init in kf: " << Gmap.AnchorsDATA[idx_new_pt].init_KF << " matched in kf: " << idx_kf_target << endl;

                    } //  for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table    
                 
                 } // for(unsigned int k =0 ; k< matches.size();k++ ) 
              } // if(idx_kf_target >= idx_Fixed_kf_ref)      

      } // for (int i = 0; i < idx_linked_KF.size() ; i++)


      int q = 10;


  }// if (Gmap.KeyFDATA.size() > 2)



}









//---------------------------------------------------------------------------------------------------------
#define RATIO    0.85
//#define RATIO    0.85
void matchFeatures3(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
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