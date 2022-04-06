#include "ekf.hpp"
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Transforms/AngleWrap.hpp"

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
void EKF::Attitude_euler_Update(ATT &att)
{

    double phi = att.roll; 
    double theta = att.pitch;
    double psi = att.yaw - yaw_at_home;  //  use local yaw measurements
    //double psi = att.yaw;  //  use local yaw measurements
    AngleWrap(psi);

    arma::vec::fixed<3> z;  //  measurement vector    
      z[0] = phi;
      z[1] = theta;
      z[2] = psi;       
    
      arma::vec::fixed<3> h;  // prediction vector
      h[0] = x[1];
      h[1] = x[2];
      h[2] = x[3];
     
      // For debug
      //cout << "z: " << z[0] << " " << z[1] << " " << z[2] << endl;
      //cout << "h: " << h[0] << " " << h[1] << " " << h[2]  << endl;
      
      
      arma::mat H;      
      int x_len = x.size();
      H.resize(3,x_len);
      arma::mat::fixed<3,3> de_de = arma::mat(3,3,arma::fill::eye);
      H(arma::span(0,2),arma::span(1,3)) = de_de;
      
      double vs = PAR.Sigma_att_update*PAR.Sigma_att_update;  // variance
      arma::mat::fixed<3,3> R;
      R.zeros();
              R(0,0) = vs; //*.0000001;
              R(1,1) = vs; //*.0000001;
              R(2,2) = vs; //*.0000001;              
              
              arma::mat H_P = H*P;
              
              arma::mat S = H_P*H.t() + R; // Innovation matrix
                      
              arma::mat K = P*H.t()*arma::inv(S); // Kalman gain
              
              arma::vec::fixed<3> inov = z - h;
             
              arma::vec K_inov = K*inov;
              
              /*
              // for debug              
              cout << "P norm a " << arma::norm(P) << endl;
              cout << "x norm a " << arma::norm(x) << endl;              
              cout << R << endl;
              cout << H_P*H.t() << endl;
              cout << inv(S) << endl;
              cout << K*(inov) << endl;
              */

              P = P - K*H_P;  // System Covariance matrix update                              

              x = x + K_inov;  // System vector update
              

              

    

}


//------------------------------------------------------------------------------------------------
/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0:3)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to camera)
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.*/

void EKF::Attitude_quat_Update(ATT &att)
{
    
    static double last_qz = 0;
    static double last_qy = 0;
    static double last_qx = 0;

     arma::vec::fixed<4> z;  // quat measurement
    
    double axis_x = att.roll; 
    double axis_y = att.pitch;
    double axis_z = att.yaw - yaw_at_home;  //  use local yaw measurements
    //double axis_z = att.yaw;  //  use local yaw measurements
    AngleWrap(axis_z);
      
    //cout << "axis_x: " << axis_x << " axis_y: " << axis_y << " axis_z:" << axis_z << endl;
        
    double Ra2b[9];
    Euler_to_Ra2b_colum_major(axis_x, axis_y, axis_z, Ra2b);
    double q_z[4];
    Ra2b_TO_Quat_a2b(Ra2b,q_z);
    
    bool ekf_update = true;
    if(((q_z[1] > 0)&&(last_qx <0))||((q_z[1] < 0)&&(last_qx >0)))
    {
    // if change of sign in z axis, force update
      x[0] = q_z[0];
      x[1] = q_z[1];  
      x[2] = q_z[2];    
      x[3] = q_z[3];  
      ekf_update = false;
    }
    if(((q_z[2] > 0)&&(last_qy <0))||((q_z[2] < 0)&&(last_qy >0)))
    {
    // if change of sign in z axis, force update
      x[0] = q_z[0];
      x[1] = q_z[1];  
      x[2] = q_z[2];    
      x[3] = q_z[3];
      ekf_update = false;  
    }
    if(((q_z[3] > 0)&&(last_qz <0))||((q_z[3] < 0)&&(last_qz >0)))
    {
    // if change of sign in z axis, force update
      x[0] = q_z[0];
      x[1] = q_z[1];  
      x[2] = q_z[2];    
      x[3] = q_z[3];
      ekf_update = false;  
    }
    last_qx = q_z[1];
    last_qy = q_z[2];
    last_qz = q_z[3];
    
    
    
    //if(ekf_update == true)
    //{ 
      
      // vector measurement
      z[0] = q_z[0];
      z[1] = q_z[1];
      z[2] = q_z[2];
      z[3] = q_z[3];
       
    
      arma::vec::fixed<4> h;  // quat prediction
      h[0] = x[0];
      h[1] = x[1];
      h[2] = x[2];
      h[3] = x[3];

      cout << "q_z: " << z[0] << " " << z[1] << " " << z[2] << " " << z[3] << endl;

      cout << "q_h: " << h[0] << " " << h[1] << " " << h[2] << " " << h[3] << endl;

      int x_len = x.size();
      arma::mat H;
      // Form jacobian
      H.resize(4,x_len);
      arma::mat::fixed<4,4> dq_dq = arma::mat(4,4,arma::fill::eye);
      H(arma::span(0,3),arma::span(0,3)) = dq_dq;

      double vs = PAR.Sigma_att_update*PAR.Sigma_att_update;  // variance
      
      arma::mat::fixed<4,4> R;
      R.zeros();
              R(0,0) = vs; //*.0000001;
              R(1,1) = vs; //*.0000001;
              R(2,2) = vs; //*.0000001;
              R(3,3) = vs; //*.0000001; 
              
              arma::mat H_P = H*P;
              
              arma::mat S = H_P*H.t() + R; // Innovation matrix
                      
              arma::mat K = P*H.t()*arma::inv(S); // Kalman gain
             
              P = P - K*H_P;  // System Covariance matrix update     
                
              arma::vec::fixed<4> inov = z - h;                

              x = x + K*(inov);  // System vector update               
              
              //cout << "sigma: " << PAR.Sigma_att_update << endl;
              //cout << P*H.t() << endl;
              //cout << arma::inv(S) << endl;
              //cout << K << endl;
              cout << K*(inov) << endl;
   // }             

}    