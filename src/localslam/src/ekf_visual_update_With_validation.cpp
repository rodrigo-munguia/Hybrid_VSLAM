#include "ekf.hpp"
#include "ekf_Jacobians.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"



void EKF::Visual_update_With_val_e()
{
    double v = sqrt(x(10)*x(10) + x(11)*x(11)); // current velocity over the x-y plane    

    if (v > PAR.Visual_update_min_vel_for_update_feats) 
    {   
        // 1-Point RANSAC hypothesis and selection of low-innovation inliers
        std::vector<int> z_li_f;
        std::vector<int> z_li_a;
        Visual_update_One_Point_ransac_hypotheses_e(z_li_f,z_li_a);
        
        // Partial update using low-innovation inliers
        Visual_update_One_Point_ransac_e(z_li_f,z_li_a);
        
        //cout << "li: " << z_li_f.size() << " " << z_li_a.size() << endl;

        // "Rescue" high-innovation inliers
        std::vector<int> z_hi_f;
        std::vector<int> z_hi_a;
        Visual_update_One_Point_ransac_rescue_hi_inliers_e(z_li_f,z_li_a,z_hi_f,z_hi_a );

        // Partial update using high-innovation inliers
        Visual_update_One_Point_ransac_e(z_hi_f,z_hi_a);

        //cout << "hi: "<< z_hi_f.size() << " " << z_hi_a.size() << endl;
    }


}


//--------------------
// get a random number in a range
// Rodrigo M. 2022
int random(int min, int max) //range : [min, max]
{
   static bool first = true;
   if (first) 
   {  
      std::srand( time(NULL) ); //seeding for the first time only!
      first = false;
   }
   return min + std::rand() % (( max + 1 ) - min);
}
//---------------------------------------------------------------------------------
// This function returns the index of a random matched feature or anchor,
// returns "type" = 0 if feature, "type = 1" if anchor
// Rodrigo M. 2022
int select_random_match(std::vector<FEAT> &FeatsDATA, std::vector<FEAT> &AnchorsDATA, int &type)
{
    std::vector<int> matched_index;
    
    for (int i = 0; i < FeatsDATA.size(); i++)
    {
        if(FeatsDATA[i].matched == true)
        {
          matched_index.push_back(i);  
        }
    }
    int max_idx_feats = matched_index.size()- 1; // store the size vector "matched_index" corresponding to features index
    for (int i = 0; i < AnchorsDATA.size(); i++)
    {
        if(AnchorsDATA[i].matched == true)
        {
          matched_index.push_back(i);  
        }
    }

    int rnd_idx = random(0,matched_index.size()-1);

    if(rnd_idx > max_idx_feats)
    {
        type = 1; // it is an anchor
    }
    else
    {
        type = 0; // it is a feature
    }
    return matched_index[rnd_idx];
}
//--------------------------------------------------------------------------------
// This function returns the Jacobian "Hi", measurement vector "zi", prediction vector "hi" and measurement noise matrix "Ri" for a single feat/anchor
// Rodrigo M. 2022
void FeatPrediction(arma::vec &x, std::vector<FEAT> &FeatsDATA, std::vector<FEAT> &AnchorsDATA, int type, int idx, CAM &camera_parameters, parameters &par, arma::mat &Rr2c,arma::vec::fixed<3> &t_c2r, arma::vec::fixed<2> &hi, arma::vec::fixed<2> &zi, arma::mat::fixed<2,2> &Ri, arma::mat &Hi )
{                        
            int x_len = x.size();
            int idx_i = FeatsDATA[idx].idx_i_state;
            int idx_f = FeatsDATA[idx].idx_f_state;
            arma::vec::fixed<3> pxyz = x.subvec(idx_i,idx_f);            
            arma::mat::fixed<2,13> duv_dx;
            arma::mat::fixed<2,3> duv_dy;              
            Jac_e_uv_XYZ(pxyz,x,camera_parameters,Rr2c,t_c2r,duv_dx,duv_dy,par.Visual_update_attitude_update);            
            Hi.zeros(2,x_len);
            if(type == 0) // for features
            { 
                Hi(arma::span(0,1),arma::span(0,12)) = duv_dx;
                Hi(arma::span(0,1),arma::span(idx_i,idx_f)) = duv_dy; 
                FeatsDATA[idx].duv_dx = duv_dx;
                FeatsDATA[idx].duv_dy = duv_dy;
                hi(0) = FeatsDATA[idx].PredictedPoint.x;
                hi(1) = FeatsDATA[idx].PredictedPoint.x;
                zi(0) = FeatsDATA[idx].MatchedPoint.x;
                zi(1) = FeatsDATA[idx].MatchedPoint.y;
                Ri.zeros();
                Ri(0,0) = pow(par.Sigma_cam_uv,2);
                Ri(1,1) = pow(par.Sigma_cam_uv,2);

            }
            else if(type == 1)
            {
                Hi(arma::span(0,1),arma::span(0,12)) = duv_dx;
                AnchorsDATA[idx].duv_dx = duv_dx;
                hi(0) = AnchorsDATA[idx].PredictedPoint.x;
                hi(1) = AnchorsDATA[idx].PredictedPoint.x;
                zi(0) = AnchorsDATA[idx].MatchedPoint.x;
                zi(1) = AnchorsDATA[idx].MatchedPoint.y;
                Ri.zeros();
                Ri(0,0) = pow(par.Sigma_cam_uv_anchors,2);
                Ri(1,1) = pow(par.Sigma_cam_uv_anchors,2);
            }    

}

//--------------------------------------------------------------------------------
// This function returns the predicted visual measurement (hi) for the idx-feature/anchor
// Rodrigo M. 2022
arma::vec::fixed<2> FeatPrediction_only_hi(arma::vec &xi, std::vector<FEAT> &FeatsDATA,std::vector<FEAT> &AnchorsDATA,int type, int idx,CAM &camera_parameters, parameters &par, arma::mat &Rc2r,arma::vec::fixed<3> &t_c2r)
{   
    arma::vec::fixed<2> hi;
    if(type == 0) // compute for feature
    {
            int idx_i = FeatsDATA[idx].idx_i_state;
            int idx_f = FeatsDATA[idx].idx_f_state;            
            double phi = xi(1);
            double theta = xi(2);
            double psi = xi(3);
            double Ra2b[9];
            Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
            arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
            arma::mat::fixed<3,3> Rr2n = Rn2r.t();                    
            arma::mat::fixed<3,3> Rc2n = Rr2n*Rc2r;
            arma::mat::fixed<3,3> Rn2c = Rc2n.t();
            arma::vec::fixed<3> Tn2c = xi.subvec(7,9) + Rr2n*t_c2r;  // Tn2c = r^N + Rr2n*Tc2r
            arma::vec::fixed<3> pxyz = xi.subvec(idx_i,idx_f);            
            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame            
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,camera_parameters,duv_dPc );
            hi(0) = uvd.x;
            hi(1) = uvd.y;
            //cout << uvd << endl;

    }
    else if(type == 1) // compute for anchor
    {       
            double phi = xi(1);
            double theta = xi(2);
            double psi = xi(3);
            double Ra2b[9];
            Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
            arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
            arma::mat::fixed<3,3> Rr2n = Rn2r.t();                 
            arma::mat::fixed<3,3> Rc2n = Rr2n*Rc2r;
            arma::mat::fixed<3,3> Rn2c = Rc2n.t();            
            arma::vec::fixed<3> Tn2c = xi.subvec(7,9) + Rr2n*t_c2r;  // Tn2c = r^N + Rr2n*Tc2r
            arma::vec::fixed<3> pxyz = AnchorsDATA[idx].AnchorState;     
            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,camera_parameters,duv_dPc );
            hi(0) = uvd.x;
            hi(1) = uvd.y;
            //cout << uvd << endl;
    }

    return hi;

}

//--------------------------------------------------------------------------------
// This function returns the index of features and anchors (z_th_f and z_th_f) whose innovation is below a treshold
// Also returns the mumber of matches (num_IC_matches) and a number for hypothesis support (num_IC_matches)
// Rodrigo M. 2022
void Find_matches_below_a_treshold(arma::vec &xi,std::vector<FEAT> &FeatsDATA, std::vector<FEAT> &AnchorsDATA, CAM &camera_parameters, parameters &par,arma::mat &Rc2r,arma::vec::fixed<3> &t_c2r,int &hypothesis_support, std::vector<int> &z_th_f,std::vector<int> &z_th_a, int &num_IC_matches, double &threshold_f,double &threshold_a  )
{
    hypothesis_support = 0;
    num_IC_matches = 0;
    
    // for feats
    for(int i = 0; i < FeatsDATA.size(); i++)
    {
        if(FeatsDATA[i].matched == true)
        {   
            num_IC_matches++; // count number of matches for his frame

            arma::vec::fixed<2> hi;
            hi = FeatPrediction_only_hi(xi, FeatsDATA,AnchorsDATA,0, i,camera_parameters, par,Rc2r,t_c2r);           
                      
            
            arma::vec::fixed<2> zi;
            zi(0) = FeatsDATA[i].MatchedPoint.x;
            zi(1) = FeatsDATA[i].MatchedPoint.y;

            arma::vec::fixed<2> inov = zi-hi;            
            
            double inov_s = inov(0)*inov(0) + inov(1)*inov(1);

            if ( sqrt(inov_s) < threshold_f )
            {
                hypothesis_support++;
                z_th_f.push_back(i);
            }
                      
            
        }
    }
   
    // for anchors
    
    for(int i = 0; i < AnchorsDATA.size(); i++)
    {
        if(AnchorsDATA[i].matched == true)
        {   
            num_IC_matches++; // count number of matches for his frame

            arma::vec::fixed<2> hi;
            hi = FeatPrediction_only_hi(xi, FeatsDATA,AnchorsDATA,1, i,camera_parameters, par,Rc2r,t_c2r);
            
            
             arma::vec::fixed<2> zi;
            zi(0) = AnchorsDATA[i].MatchedPoint.x;
            zi(1) = AnchorsDATA[i].MatchedPoint.y;

            arma::vec::fixed<2> inov = zi-hi;            
            
            double inov_s = inov(0)*inov(0) + inov(1)*inov(1);

            if ( sqrt(inov_s) < threshold_a )
            {
                hypothesis_support++;
                z_th_a.push_back(i);
            }
        }
    } 


}
//--------------------------------------------------------------------------------
// 1-Point RANSAC: hypothesis and selection of low-innovation inliers
// returns the index of features and anchors (z_li_f and z_li_a)  
// Method adapted from paper: "1-Point RANSAC for EKF Filtering. Application to Real-Time Structure from Motion and Visual Odometry"
// to include visual updates from anchors
// Rodrigo M. 2022
void EKF::Visual_update_One_Point_ransac_hypotheses_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a)
{  
         
  arma::mat::fixed<3,3> Rc2r = Rr2c.t();   


   double p_at_least_one_spurious_free = 0.99; // default value 
   // RANSAC threshold should have a low value (less than the standard
   // deviation of the filter measurement noise); as high innovation points
   // will be later rescued
   double threshold_f = PAR.Sigma_cam_uv/1.2;
   double threshold_a = PAR.Sigma_cam_uv_anchors/1.2;

   int n_hyp = 100; //1000 % initial number of iterations, will be updated
   int max_hypothesis_support = 0; // will be updated
    
   for (int i = 0; i < n_hyp; i++)
   {   
        int type;
        int rnd_idx = select_random_match(FeatsDATA,AnchorsDATA,type); // get index of a random matched feat/anchor 
        //cout << rnd_idx << "  " << type << " N feats: " << FeatsDATA.size() << " N anchors: " << AnchorsDATA.size()<< endl;

       arma::vec::fixed<2> hi;
       arma::vec::fixed<2> zi;
       arma::mat::fixed<2,2> Ri;
       arma::mat Hi;
       FeatPrediction(x,FeatsDATA,AnchorsDATA,type, rnd_idx, cam_parameters, PAR,Rr2c,t_c2r, hi, zi,Ri,Hi );
        
       arma::mat H_P = Hi*P;
       arma::mat Si = H_P*Hi.t() + Ri; // Innovation matrix        
       arma::mat K = P*Hi.t()*arma::inv(Si); // Kalman gain       
       arma::vec xi = x; 
       xi = xi + K*(zi-hi);  // System vector update
       
       int hypothesis_support;
       std::vector<int> z_th_f; // index of matched features below a treshold
       std::vector<int> z_th_a; // index of matched anchors belos a trshold
       int num_IC_matches; 
       
       
      Find_matches_below_a_treshold(xi,FeatsDATA,AnchorsDATA,cam_parameters,PAR,Rc2r,t_c2r,hypothesis_support, z_th_f,z_th_a, num_IC_matches ,threshold_f,threshold_a  );

     // cout << "num_IC_matches: " << num_IC_matches << " hypothesis_support: " << hypothesis_support << endl;

      if( hypothesis_support > max_hypothesis_support )
      {
          max_hypothesis_support = hypothesis_support;
          
          z_li_f = z_th_f;
          z_li_a = z_th_a;  

          double epsilon = 1 -((double)hypothesis_support/(double)num_IC_matches);

          n_hyp = ceil( log(1- p_at_least_one_spurious_free)/log(1-(1-epsilon)) );

         // cout << epsilon << endl; 
         // cout << n_hyp << endl;

          if (n_hyp == 0)
          {
              break;
          }  

      }  


   }
}

//--------------------------------------------------------------------------------
// 1-Point RANSAC: update system state using inliers amtches
// Method adapted from paper: "1-Point RANSAC for EKF Filtering. Application to Real-Time Structure from Motion and Visual Odometry"
// to include visual updates from anchors
// Rodrigo M. 2022
void EKF::Visual_update_One_Point_ransac_e(std::vector<int> &z_lx_f,std::vector<int> &z_lx_a)
{
    int x_len = x.size();
    arma::mat H;
    arma::vec z;
    arma::vec h;
    arma::mat R;         

    // for feats
    for(int i = 0; i < z_lx_f.size(); i++)
    { 
        int i_feat = z_lx_f[i];
        int idx_i = FeatsDATA[i_feat].idx_i_state;
        int idx_f = FeatsDATA[i_feat].idx_f_state;
        arma::vec::fixed<3> pxyz = x.subvec(idx_i,idx_f);                
        arma::mat::fixed<2,13> duv_dx;
        arma::mat::fixed<2,3> duv_dy;
        Jac_e_uv_XYZ(pxyz,x,cam_parameters,Rr2c,t_c2r,duv_dx,duv_dy,PAR.Visual_update_attitude_update);
        arma::mat hi;
        hi.zeros(2,x_len);
        hi(arma::span(0,1),arma::span(0,12)) = duv_dx;
        hi(arma::span(0,1),arma::span(idx_i,idx_f)) = duv_dy;        
        int idx = H.n_rows;
        // Form jacobian
        H.resize(idx+2,x_len);
        H(arma::span(idx,idx+1),arma::span(0,x_len-1)) = hi;   
        // Form prediction vector
        h.resize(idx+2);  
        h(idx) = FeatsDATA[i_feat].PredictedPoint.x;
        h(idx+1) = FeatsDATA[i_feat].PredictedPoint.y;
        // Form measurement vector
        z.resize(idx+2);
        z(idx) = FeatsDATA[i_feat].MatchedPoint.x;
        z(idx+1) = FeatsDATA[i_feat].MatchedPoint.y;
        // Form measurement noise matrix
        R.resize(idx+2,idx+2);
        R(idx,idx) = pow(PAR.Sigma_cam_uv,2);
        R(idx+1,idx+1) = pow(PAR.Sigma_cam_uv,2);
    }
    // for anchors
    for(int i = 0; i < z_lx_a.size(); i++)
    {   
        int i_anchor = z_lx_a[i];
        arma::vec::fixed<3> pxyz = AnchorsDATA[i_anchor].AnchorState;
        arma::mat::fixed<2,13> duv_dx;
        arma::mat::fixed<2,3> duv_dy;            
        Jac_e_uv_XYZ(pxyz,x,cam_parameters,Rr2c,t_c2r,duv_dx,duv_dy,PAR.Visual_update_attitude_update); // to implement efficient jca
        arma::mat hi;
        hi.zeros(2,x_len);
        hi(arma::span(0,1),arma::span(0,12)) = duv_dx; 
        int idx = H.n_rows;
        // Form jacobian
        H.resize(idx+2,x_len);
        H(arma::span(idx,idx+1),arma::span(0,x_len-1)) = hi;   
        // Form prediction vector
        h.resize(idx+2);  
        h(idx) = AnchorsDATA[i_anchor].PredictedPoint.x;
        h(idx+1) = AnchorsDATA[i_anchor].PredictedPoint.y;
        // Form measurement vector
        z.resize(idx+2);
        z(idx) = AnchorsDATA[i_anchor].MatchedPoint.x;
        z(idx+1) = AnchorsDATA[i_anchor].MatchedPoint.y;
        // Form measurement noise matrix
        R.resize(idx+2,idx+2);
        R(idx,idx) = pow(PAR.Sigma_cam_uv_anchors,2);
        R(idx+1,idx+1) = pow(PAR.Sigma_cam_uv_anchors,2); 
    }  

    //----------------------------------------------------------------
    // Kalman Update
    int n = h.size();   
    if(n>0)
    {
        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update

    }

}


//--------------------------------------------------------------------------------
// 1-Point RANSAC: "Rescue" high-innovation inliers
// returns the index of features and anchors (z_hi_f and z_hi_a) rescued
// Method adapted from paper: "1-Point RANSAC for EKF Filtering. Application to Real-Time Structure from Motion and Visual Odometry"
// to include visual updates from anchors
// Rodrigo M. 2022
void EKF::Visual_update_One_Point_ransac_rescue_hi_inliers_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a,std::vector<int> &z_hi_f,std::vector<int> &z_hi_a )
{
  double chi2inv_2_95 = 9.2103;
  
arma::mat::fixed<3,3> Rc2r = Rr2c.t();   
   
   int n_matched_f_a = 0;
  // for feats
   for (int i = 0; i < FeatsDATA.size(); i++)
   {
      if(FeatsDATA[i].matched == true)
      {   
          n_matched_f_a++;
          // if the matched feture is not already considered as a low-innovation one
          if ( std::find(z_li_f.begin(), z_li_f.end(), i) == z_li_f.end() )
          {

              arma::vec::fixed<2> hi;
              arma::vec::fixed<2> zi;
              arma::mat::fixed<2,2> Ri;
              arma::mat Hi;
              FeatPrediction(x,FeatsDATA,AnchorsDATA,0, i, cam_parameters, PAR,Rr2c,t_c2r, hi, zi,Ri,Hi );
              arma::mat H_P = Hi*P;
              arma::mat Si = H_P*Hi.t() + Ri; // Innovation matrix
              arma::vec::fixed<2> v = zi-hi;  
              
              double si = arma::as_scalar(v.t()*arma::inv(Si)*v);

              if (si < chi2inv_2_95)
              {
                 z_hi_f.push_back(i); 
              }

          } // if ( std::find(z_li_f.begin(), z_li_f.end(), i) == z_li_f.end() )      

      } // if(FeatsDATA[i].matched == true)

    } // for (int i = 0; i < FeatsDATA.size(); i++)

    // for anchors
    for (int i = 0; i < AnchorsDATA.size(); i++)
    {
      if(AnchorsDATA[i].matched == true)
      {   
          n_matched_f_a++;            
          // if the matched feture is not already considered as a low-innovation one
          if ( std::find(z_li_a.begin(), z_li_a.end(), i) == z_li_a.end() )
          {
              arma::vec::fixed<2> hi;
              arma::vec::fixed<2> zi;
              arma::mat::fixed<2,2> Ri;
              arma::mat Hi;
              FeatPrediction(x,FeatsDATA,AnchorsDATA,1, i, cam_parameters, PAR,Rr2c,t_c2r, hi, zi,Ri,Hi );
              arma::mat H_P = Hi*P;
              arma::mat Si = H_P*Hi.t() + Ri; // Innovation matrix
              arma::vec::fixed<2> v = zi-hi;  
              
              double si = arma::as_scalar(v.t()*arma::inv(Si)*v);

              if (si < chi2inv_2_95)
              {
                 z_hi_a.push_back(i); 
              }

          } // if ( std::find(z_li_f.begin(), z_li_f.end(), i) == z_li_f.end() )      

      } // if(FeatsDATA[i].matched == true)

    } // for (int i = 0; i < FeatsDATA.size(); i++)


    //int n_updated_feats = z_li_f.size() + z_li_a.size() + z_hi_f.size() + z_hi_a.size();

    //cout << "rejected matches: " << n_matched_f_a - n_updated_feats << "/" <<  n_matched_f_a << endl;



}