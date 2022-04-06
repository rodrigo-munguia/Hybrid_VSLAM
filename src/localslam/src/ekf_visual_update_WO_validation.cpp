#include "ekf.hpp"
#include "ekf_Jacobians.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"




//----------------------------------------------------------------------------------------------------
// Function for updating system state and covariance matrix from visual (monocular) measurements 
// No method for detecting measurements outliers is applied, i.e. it takes as truth the output from the matching process
// Rodrigo M. 2021
void EKF::Visual_update_WO_val()
{
    double v = sqrt(x(10)*x(10) + x(11)*x(11)); // current velocity over the x-y plane    
    Visual_update_anchors_WO_val_e(); // update with anchors

    if (v > PAR.Visual_update_min_vel_for_update_feats)  
        Visual_update_feats_WO_val_e();  // update with feats


}

//----------------------------------------------------------------------------------------------------
// Function for updating system state and covariance matrix from visual (monocular) measurements of anchors
// Attitude of the robot is expressed by euler angles
// Rodrigo M. 2021
void EKF::Visual_update_feats_WO_val_e()
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

    int x_len = x.size();
    arma::mat H;
    arma::vec z;
    arma::vec h;
    arma::mat R;
         
    

    for(int i = 0; i< FeatsDATA.size();i++ )
    {
        if(FeatsDATA[i].matched == true)
        {

            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;

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
            h(idx) = FeatsDATA[i].PredictedPoint.x;
            h(idx+1) = FeatsDATA[i].PredictedPoint.y;
            
            // Form measurement vector
            z.resize(idx+2);
            z(idx) = FeatsDATA[i].MatchedPoint.x;
            z(idx+1) = FeatsDATA[i].MatchedPoint.y;

            // Form measurement noise matrix

            R.resize(idx+2,idx+2);
            R(idx,idx) = pow(PAR.Sigma_cam_uv,2);
            R(idx+1,idx+1) = pow(PAR.Sigma_cam_uv,2);

            
        
        }

    }
 //Hi(arma::span(0,24),arma::span(0,30)).print();

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


//----------------------------------------------------------------------------------------------------
// Function for updating system state and covariance matrix from visual (monocular) measurements of anchors
// Attitude of the robot is expressed by euler angles
// Rodrigo M. 2021
void EKF::Visual_update_anchors_WO_val_e()
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
    int n_anchors;
    int x_len = x.size();
    arma::mat H;
    arma::vec z;
    arma::vec h;
    arma::mat R;
         

    for(int i = 0; i< AnchorsDATA.size();i++)
    {
        if(AnchorsDATA[i].matched == true)
        {
            arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;            
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
            h(idx) = AnchorsDATA[i].PredictedPoint.x;
            h(idx+1) = AnchorsDATA[i].PredictedPoint.y;
            
            // Form measurement vector
            z.resize(idx+2);
            z(idx) = AnchorsDATA[i].MatchedPoint.x;
            z(idx+1) = AnchorsDATA[i].MatchedPoint.y;

            // Form measurement noise matrix

            R.resize(idx+2,idx+2);
            R(idx,idx) = pow(PAR.Sigma_cam_uv_anchors,2);
            R(idx+1,idx+1) = pow(PAR.Sigma_cam_uv_anchors,2); 



        }

    }

    //----------------------------------------------------------------
// Kalman Update
    int n = h.size();

    if(n>0)
    {
        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
        //S.print();
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update


    }

}
//------------------------------------------------------------------------------------------------------------------------
