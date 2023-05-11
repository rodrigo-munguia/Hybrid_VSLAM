#include "ekf.hpp"
#include "ekf_Jacobians.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"


void matchFeatures2b(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches );
//----------------------------------------------------------------------------------------------------
// Function for initialize new euclidean (xyz) features into the system state from visual measurements s
// In the state vector, the attitude of the robot must be expressed by euler angles
// Rodrigo M. 2022
void EKF::Visual_update_e_init_feat_delayed(FRAME *frame)
{
   /*
% Initial States ***************************************************
% x meaning
% index  0    1   2    3   4   5   6    7  8  9  10  11  12  
%       null phi theta psi w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0) not used
% x(1:3)=   [phi theta psi] -> roll, pitch, and yaw of the robot
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in body coordinate frame.
*/

static bool init = false;
static cv::Mat old_frame;
static cv::Mat old_Descriptors;
static vector<cv::KeyPoint> old_Points;
static arma::vec::fixed<13> old_x_r;
static arma::mat::fixed<3,3> Rn2c_old;
static arma::vec::fixed<3> t_c2n_old;

   if(init == false)
   {
      init = true;
      old_frame = frame->image;      
      vector<cv::Mat> Patches_init;
      vector<cv::Mat> Patches_match;
      Get_img_points_for_init_feats(frame, old_Points, old_Descriptors, Patches_init,Patches_match);
      old_x_r = x.subvec(0,12);
      
      double phi = x(1);
      double theta = x(2);
      double psi = x(3);
      double Ra2b[9];
      arma::mat::fixed<3,3> Rc2r = Rr2c.t();
      Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
      arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
      arma::mat::fixed<3,3> Rr2n = Rn2r.t();
      arma::mat Rc2n = Rr2n*Rc2r;    
      Rn2c_old = Rc2n.t();
      t_c2n_old = x.subvec(7,9) + Rn2r.t()*t_c2r;
                       
   }
   else
   {
      // check for robot displasment
      
      double d = arma::norm(x.subvec(7,9) - old_x_r.subvec(7,9), 2  );

      // if the robot has a minimun displasment then try to triangulate 
      if(d > .1)
      {  
         cv::Mat new_Descriptors;
         vector<cv::KeyPoint> new_Points;
         vector<cv::Mat> Patches_init;
         vector<cv::Mat> Patches_match;
         Get_img_points_for_init_feats(frame, new_Points, new_Descriptors, Patches_init,Patches_match);
         
         std::vector<cv::DMatch> matches;      
         matchFeatures2b(old_Descriptors, new_Descriptors, matches);

         // -----------  Use RANSAC (Find Fundamental Matrix) to discard outliers
         std::vector<cv::Point2f> Points_m2;
         std::vector<cv::Point2f> Points_m1;
         for(unsigned int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
            {  
               int Descriptor_idx_m2 = matches[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
               int Descriptor_idx_m1 = matches[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
               cv::Point2f point_idx_m2 = old_Points[Descriptor_idx_m2].pt;
               cv::Point2f point_idx_m1 = new_Points[Descriptor_idx_m1].pt;
               Points_m2.push_back(point_idx_m2);
               Points_m1.push_back(point_idx_m1); 
            }      
         cv::Mat status;
         //cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);
         if(Points_m2.size() > 0)
            cv::Mat H = cv::findFundamentalMat(Points_m2, Points_m1, cv::RANSAC, 3, 0.99, status );
         // obtain inliers
         std::vector<cv::DMatch> Matches_final;   // Store correct matches in "inliers"
         if(status.rows > 0)
            for(size_t i = 0; i < Points_m2.size(); i++) 
                  {
                     if(status.at<char>(i) != 0) 
                     {
                        Matches_final.push_back(matches[i]);
                     }
                  } 
         //----------------------------------------------------------------------------
         
         double phi = x(1);
         double theta = x(2);
         double psi = x(3);
         double Ra2b[9];
         arma::mat::fixed<3,3> Rc2r = Rr2c.t();
         Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
         arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
         arma::mat::fixed<3,3> Rr2n = Rn2r.t();
         arma::mat Rc2n = Rr2n*Rc2r;    
         arma::mat::fixed<3,3> Rn2c_new = Rc2n.t();
         arma::vec::fixed<3> t_c2n_new = x.subvec(7,9) + Rn2r.t()*t_c2r;

         //--------------------------------------------------------  
         // Triangulation by Simple triangulation method -------------------------

       //cout << "Simple triangulate:" << endl; 
        for(unsigned int i =0 ; i< Matches_final.size();i++ ) // for each match, update Feature table
        {
          int idx_descriptor_previous = Matches_final[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int idx_descriptor_current = Matches_final[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f uv2d = old_Points[idx_descriptor_previous].pt;
          cv::Point2f uv1d = new_Points[idx_descriptor_current].pt;   
        
          arma::vec::fixed<3> pt_n = Triangulate_sigle_3d_point(uv1d,uv2d,Rn2c_new,t_c2n_new,Rn2c_old,t_c2n_old );         
          //cout << pt_n.t() << endl;
          
               if(!(pt_n[0] == 0 && pt_n[1] == 0 && pt_n[2] == 0) )
               {  
                  double depth = arma::norm(x.subvec(7,9) - pt_n, 2  );
                  
                  if(depth > .2){

                     arma::vec::fixed<3> new_yi = pt_n;
                     // vector state augmentation
                     int old_x_size = x.size();      
                     x.resize( old_x_size +3 ); // resize state vector
                     int new_x_size = x.size();
                     x.subvec(old_x_size,new_x_size-1) = new_yi; // augmentate state with new feature state

                     double sig_ini = PAR.Sigma_cam_initial_depth_WOrange; // Initial hypotheses of depth

                     //---- get Initialization Jacobians
                     arma::mat::fixed<3,13> dy_dx;  
                     arma::mat::fixed<3,3> dy_duvr; 
                     Jac_e_init_feat_func(uv1d, x, cam_parameters,Rc2r,t_c2r, depth,dy_dx, dy_duvr);
                  
                     arma::mat dh_dx;
                     dh_dx = zeros(3,old_x_size);
                     dh_dx(arma::span(0,2),arma::span(0,12)) = dy_dx;

                     arma::mat::fixed<3,3> Ry;
                     Ry.zeros();
                     Ry(0,0) = pow(PAR.Sigma_cam_uv,2);
                     Ry(1,1) = pow(PAR.Sigma_cam_uv,2);
                     Ry(2,2) = pow(sig_ini,2);

                     arma::mat dh_dxP = dh_dx*P;      
                     arma::mat::fixed<3,3> Py =  dh_dxP(arma::span(0,2),arma::span(0,12))*dy_dx.t() + dy_duvr*Ry*dy_duvr.t(); 

                     P.resize( old_x_size +3, old_x_size +3); // resize covariance matrix

                     P(arma::span(old_x_size,new_x_size-1),arma::span(old_x_size,new_x_size-1)) = Py;    
                     P(arma::span(old_x_size,new_x_size-1),arma::span(0,old_x_size-1)) = dh_dxP;
                     P(arma::span(0,old_x_size-1),arma::span(old_x_size,new_x_size-1)) = dh_dxP.t();

                     //cout << P << endl;
                  
                     // ---- Update feature vector 
                     
                     FEAT feat;  // feature data structure
                     feat.feat_type = "XYZ";
                     feat.idx_i_state = old_x_size;
                     feat.idx_f_state = new_x_size-1;
                     feat.Initial_KeyPoint = new_Points[idx_descriptor_current];
                     feat.Keypoint = new_Points[idx_descriptor_current];
                     feat.Descriptor = new_Descriptors.row(idx_descriptor_current);
                     feat.times_mathed = 0;
                     feat.times_not_mathed = 0;
                     feat.times_not_considered = 0;
                     feat.CameraState = x.subvec(7,9);

                     FeatsDATA.push_back(feat);

                     if(PAR.Stats)store.n_init_feats++;

                  }    

                  int qq = 10;
  
               }         
                  
         }

         old_frame = frame->image;
         old_Descriptors = new_Descriptors;
         old_Points = new_Points;
         old_x_r = x.subvec(0,12);
         Rn2c_old = Rn2c_new;
         t_c2n_old = t_c2n_new;
      }
      
      
   } // else


}


//----------------------------------------------------------------------------------------------------
// Function for initialize new euclidean (xyz) features into the system state from visual measurements with associated range measurements
// In the state vector, the attitude of the robot must be expressed by euler angles
// Rodrigo M. 2021
void EKF::Visual_update_e_init_feat_wr(FRAME *frame)
{
/*
% Initial States ***************************************************
% x meaning
% index  0    1   2    3   4   5   6    7  8  9  10  11  12  
%       null phi theta psi w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0) not used
% x(1:3)=   [phi theta psi] -> roll, pitch, and yaw of the robot
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in body coordinate frame.
*/
   
   vector<cv::KeyPoint> Points;
   cv::Mat Descriptors;
   vector<cv::Mat> Patches_init;
   vector<cv::Mat> Patches_match;
   Get_img_points_for_init_feats(frame, Points, Descriptors, Patches_init,Patches_match);

   arma::mat::fixed<3,3> Rc2r = Rr2c.t();

         //cout << "init R" << endl;
         //cout << cam_axis_x << " " << cam_axis_y << " " << cam_axis_z << endl;
         //cout << Rc2r << endl;
           

   for (unsigned int i =0 ; i< Points.size(); i++)
     {

         cv::Point2d uvd;
         uvd = Points[i].pt;
         arma::vec::fixed<3> hc;
         arma::mat::fixed<3,2> dhc_duvd;         

         hc = Inverse_projection_model(uvd,1,false, cam_parameters,dhc_duvd); // compute inverse projection model

         //cout << cam_parameters.distortions[0] << "  " << cam_parameters.distortions[1] << endl;
         // cout << "hc: " << hc << endl;
    
         double phi = x(1);
         double theta = x(2);
         double psi = x(3);
         double Ra2b[9];
         Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
         arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
         arma::mat::fixed<3,3> Rr2n = Rn2r.t();
        

         arma::mat Rc2n = Rr2n*Rc2r; 
        
         arma::vec::fixed<3> hn = Rc2n*hc;   //  

         arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature

         //-- compute feature depth from range measurement
         arma::vec::fixed<3> v = {0,0,1};
         arma::vec::fixed<3> vn =  Rc2n*v;
         double depth = (frame->range + PAR.Range_sensor_offset) /arma::dot(m,vn);   
        //---------         

        arma::vec::fixed<3> d_v =  depth*m;    //  d_v = Rr2n*Rc2r*P_c

        //           P_n = Rr2n*Rc2r*P_c +  (Rr2n*t_c2r + t_r2n)
        arma::vec new_yi =  d_v    +      ( Rr2n*t_c2r  + x.subvec(7,9));   // compute new Euclidean (XYZ) feature map 
       
       //cout << new_yi << endl;      
       
       // vector state augmentation
       int old_x_size = x.size();      
       x.resize( old_x_size +3 ); // resize state vector
       int new_x_size = x.size();
       x.subvec(old_x_size,new_x_size-1) = new_yi; // augmentate state with new feature state
       
       // Compute the initial uncertainty hypotheses based on the area covered by the range sensor
       // See: "Monocular SLAM System for MAVs Aided with Altitude and Range Measurements: a GPS-free Approach"       
       //-- compute radious in the image covered by the range sensor 
         double ce = PAR.Range_sensor_max_range_pattern/pow(PAR.Range_sensor_max_r_max,2);
         double r_at_c = sqrt(frame->range/ce);
         arma::vec::fixed<3> Pc = {r_at_c,0,frame->range};
         arma::mat::fixed<2,3> duv_dPc;
         cv::Point2d uvd_r = Projection_model(Pc,1,false,cam_parameters,duv_dPc );
         double r_range_image = uvd_r.x -  PAR.Mono_cam_cc_u;
       //--- compute if the visual point is inside the area of measured by the range senor 
         cv::Point2d uv;
         uv = Undistort_a_point(uvd,cam_parameters,1);
         double dp = sqrt(  pow(PAR.Mono_cam_cc_u - uv.x,2) + pow(PAR.Mono_cam_cc_u - uv.y,2) );
         
         double sig_ini; // Initial hypotheses of depth
         if( (dp < r_range_image)&&(frame->range < PAR.Range_sensor_max_range_operation ))
         {            
            if (FeatsDATA.size() > 0 )
            { 
               sig_ini =  PAR.Sigma_cam_initial_depth_range; 
            }
            else
            {  // for initial features
               sig_ini = PAR.Sigma_cam_initial_depth_range*10; 
            }  
         }
         else
         {   
            if (FeatsDATA.size() > 0 )
            { 
               sig_ini = PAR.Sigma_cam_initial_depth_WOrange;    // 5
            }
            else
            {  // for initial features
               sig_ini = PAR.Sigma_cam_initial_depth_WOrange*10;  
            } 
         }
       //------------------------------------------------------------------------------------------------  
         
       
       //---- get Initialization Jacobians
       arma::mat::fixed<3,13> dy_dx;  
       arma::mat::fixed<3,3> dy_duvr; 
       Jac_e_init_feat_func(uvd, x, cam_parameters,Rc2r,t_c2r, depth,dy_dx, dy_duvr);
     
      arma::mat dh_dx;
      dh_dx = zeros(3,old_x_size);
      dh_dx(arma::span(0,2),arma::span(0,12)) = dy_dx;

      arma::mat::fixed<3,3> Ry;
      Ry.zeros();
      Ry(0,0) = pow(PAR.Sigma_cam_uv,2);
      Ry(1,1) = pow(PAR.Sigma_cam_uv,2);
      Ry(2,2) = pow(sig_ini,2);

      arma::mat dh_dxP = dh_dx*P;      
      arma::mat::fixed<3,3> Py =  dh_dxP(arma::span(0,2),arma::span(0,12))*dy_dx.t() + dy_duvr*Ry*dy_duvr.t(); 

      P.resize( old_x_size +3, old_x_size +3); // resize covariance matrix

      P(arma::span(old_x_size,new_x_size-1),arma::span(old_x_size,new_x_size-1)) = Py;    
      P(arma::span(old_x_size,new_x_size-1),arma::span(0,old_x_size-1)) = dh_dxP;
      P(arma::span(0,old_x_size-1),arma::span(old_x_size,new_x_size-1)) = dh_dxP.t();

      //cout << P << endl;
     
      // ---- Update feature vector 
      
      FEAT feat;  // feature data structure
      feat.feat_type = "XYZ";
      feat.idx_i_state = old_x_size;
      feat.idx_f_state = new_x_size-1;
      feat.Initial_KeyPoint = Points[i];
      feat.Keypoint = Points[i];
      feat.Descriptor = Descriptors.row(i);
      feat.times_mathed = 0;
      feat.times_not_mathed = 0;
      feat.times_not_considered = 0;
      feat.CameraState = x.subvec(7,9);

      FeatsDATA.push_back(feat); 

      if(PAR.Stats)store.n_init_feats++;
      
     }// for (unsigned int i =0 ; i< Points.size(); i++)    

  // cout << "range: " << frame->range << " range type: " << frame->range_type << endl;
  

}




//------------------------------------------------------------------------------------
void EKF::Visual_update_e_init_anchors()
{
    static int id_anchor_count=0; 

   // Check state-features convergence to initialize nwe anchors
    if (FeatsDATA.size() > 0)
    {
        
        for(int i = 0; i < FeatsDATA.size(); i++ )
        {

            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;
                            
            double depth =   arma::norm(FeatsDATA[i].CameraState - x.subvec(idx_i,idx_f));

            double r_uncertanty = sqrt( 9*P(idx_i,idx_i)  + 9*P(idx_i+1,idx_i+1) + 9*P(idx_i+2,idx_i+2) );

            if (r_uncertanty/depth < PAR.Visual_update_uncertainty_depth_anchor_init)
            {
                // Add anchor to table, an delete feature from the state
                FEAT Anchor;  // feature data structure 
                Anchor = FeatsDATA[i];
                Anchor.AnchorState = x.subvec(idx_i,idx_f);

                //Anchor.AnchorState(2) =  Anchor.AnchorState(2) - .2;

                Anchor.idx_i_state = -1;
                Anchor.idx_f_state = -1;                
                Anchor.id_anchor = id_anchor_count;
                
                
                AnchorsDATA.push_back(Anchor);

                id_anchor_count++; 
                   
                Delete_i_feat_(i);
                i--;

                if(PAR.Stats)store.n_init_anchors++;  
                
            }               


        }
    }  


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