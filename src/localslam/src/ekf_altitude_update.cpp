
#include "ekf.hpp"
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


void EKF::Altitude_update(ALT &alt)
{

    static bool f_initialized = false;
    static double alt_at_home;

    double measured_alt = alt.altitude*PAR.Alt_h_2_d;
   
   if(f_initialized == false)
    {
        alt_at_home = measured_alt;
        f_initialized = true;
    }
    if(f_initialized == true)
    {        
        double z =  measured_alt -  alt_at_home;     

        int x_len = x.size();
        arma::mat H;
        // Form jacobian
        H.resize(1,x_len);
        H(0,9) = 1; 
        
        double h = x(9);
        double r = PAR.Sigma_alt_update;        

        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + r*r; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
        arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update

        int q = 10;

    }    




}