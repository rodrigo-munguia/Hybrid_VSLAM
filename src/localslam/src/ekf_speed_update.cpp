#include "ekf.hpp"
#include "../../common/Transforms/quat2R.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"


typedef std::vector<double> vec_t;

void EKF::Speed_Update_nav_euler(SPD &spd)
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

// Speed measurements must be expressed in the robot coordinate frame! 
    arma::vec::fixed<3> sp;  // measurement  
    sp(0) = spd.speedX;  
    sp(1) = spd.speedY;
    sp(2) = spd.speedZ;   

    arma::vec::fixed<3> z = PAR.Rot_spd_2_r*sp; 

    arma::vec::fixed<3> h;  // prediction vector   
      h[0] = x[10];
      h[1] = x[11];
      h[2] = x[12];
    
    //cout << "spd_x: " << z[0] << " spd_y: " << z[1] << " spd_z:" << z[2] << endl;

     arma::mat H;      
     int x_len = x.size();
     H.resize(3,x_len);
     arma::mat::fixed<3,3> dv_dv = arma::mat(3,3,arma::fill::eye);
     H(arma::span(0,2),arma::span(10,12)) = dv_dv;

    double vs = PAR.Sigma_spd_update*PAR.Sigma_spd_update;  // variance
    arma::mat::fixed<3,3> R;
    R.zeros();
    R(0,0) = vs;
    R(1,1) = vs;
    R(2,2) = vs;

    arma::mat H_P = H*P;

    arma::mat S = H_P*H.t() + R; // Innovation matrix
            //arma::mat K = P*H.t()*arma::inv_sympd(S);
    arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

    P = P - K*H_P;  // System Covariance matrix update 
    
    x = x + K*(z-h);  // System vector update

}



//----------------------------------------------------------------------------------------
void EKF::Speed_Update2(SPD &spd)
{
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
    % x(10:12)= [v_x v_y v_z]  -> Velocity in NAVIGATION! coordinate frame.*/

    arma::vec::fixed<3> z;  // measurement  
    z(0) = -spd.speedX;  
    z(1) = spd.speedY;
    z(2) = spd.speedZ;

    cout << "spd_x: " << spd.speedX << " spd_y: " << spd.speedY << " spd_z:" << spd.speedZ << endl;

    std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2r_a[9];
    quat2R_col_major(&quat[0],Rn2r_a);
    arma::mat Rn2r(Rn2r_a,3,3); // convert from arrat to armadillo 
    arma::mat::fixed<3,3>  Rr2n = Rn2r.t();
    arma::vec::fixed<3> s_c = x.subvec(10,12); // (velocity) speed vector in navigation frame     
    
    arma::vec::fixed<3> h = Rr2n*s_c;  // measurement prediction    

    //--------------- Measurement Jacobian
    int x_len = x.size();
    arma::mat H;
    // Form jacobian
    H.resize(3,x_len);
    arma::mat::fixed<3,4> dPc_dq; 
    double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    double q4 = x(3); 
    double Pn1 = x(10);  // robot velocity in x (nav frame)
    double Pn2 = x(11);  // robot velocity in y (nav frame)
    double Pn3 = x(12);  // robot velocity in z (nav frame)

   dPc_dq = {{ 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn2*q4 - 2*Pn3*q3 - 2*Pn1*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4},
                  { 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn3*q2 - 2*Pn2*q1 - 2*Pn1*q4},
                  { 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn1*q3 - 2*Pn2*q2 - 2*Pn3*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4}};

    //H(arma::span(0,2),arma::span(0,3)) = dPc_dq;
    H(arma::span(0,2),arma::span(10,12)) = Rr2n;
   //-------------------------------------------- 
    
   double vs = PAR.Sigma_spd_update*PAR.Sigma_spd_update;  // variance
   arma::mat::fixed<3,3> R;
   R.zeros();
   R(0,0) = vs;
   R(1,1) = vs;
   R(2,2) = vs;


    arma::mat H_P = H*P;

    arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
    arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

   P = P - K*H_P;  // System Covariance matrix update 
 
   x = x + K*(z-h);  // System vector update




}