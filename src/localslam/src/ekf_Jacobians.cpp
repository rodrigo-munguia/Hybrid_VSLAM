#include "ekf_Jacobians.hpp"
#include "../../common/Vision/vision.hpp"

//#include "../../common/Vision/vision.hpp"

//--------------------------------------------------------------------------------------------
using namespace std;



void Jac_euler_noise_driven_model(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu)
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
    // NOTE: this jacobian is defined with respect  to continous equations, so matrix P must be integrated
    // Pdot = Fx*P + P*Fx' + Fu*U*Fu';
    // P = P + Pdot*dt;
 
   //cout << x << endl;

    double kr = 1/Tau_r;
    double kp = 1/Tau_p;  
    // Attitude Equations
    double phi = x[1];
    double theta = x[2];
    double psi = x[3];
    double p = x[4];
    double q = x[5];
    double r = x[6];
    double u = x[10];
    double v = x[11];
    double w = x[12];
    // body rotational velocities to euler velocities Rotation matrix   
   arma::mat::fixed<3,3> R_b2e = {{1 ,  sin(phi)*tan(theta) ,  cos(phi)*tan(theta) },
                                  {0 ,       cos(phi)       ,    -sin(phi)         },
                                  {0 ,  sin(phi)/cos(theta) ,  cos(phi)/cos(theta) }};
   /*                                
   euler_dot = R_b2e*[w_x w_y w_z]'; 
   
   w_dot = -kr*[w_x w_y w_z]' +  noise;
   */
   // Position Equations
   double Ra2b[9];
   Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
   arma::mat Rn2r(Ra2b,3,3); // navigation to robot rotation matrix
   arma::mat::fixed<3,3> Rr2n = Rn2r.t();
   /*
   pos_dot = Rb2w*[v_x v_y v_z]';
   
   vel_dot = -kr*[v_x v_y v_z]' + noise;
   
   */
   // JFx = [ 0  0       0      0       0    ]
   //       [ 0 dfe_de  dfe_dw  0       0    ]
   //       [ 0  0     dfw_dw   0       0    ]
   //       [ 0 dfp_de   0      0     dfp_dv ]
   //       [ 0  0       0      0     dfv_dv ]   
   
   arma::mat::fixed<3,3> dfe_de = {{    q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta),               r*cos(phi)*(tan(theta)*tan(theta) + 1) + q*sin(phi)*(tan(theta)*tan(theta) + 1), 0},
                                   {                        - r*cos(phi) - q*sin(phi),                                                                           0, 0},
                                   {(q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta), (r*cos(phi)*sin(theta))/(cos(theta)*cos(theta)) + (q*sin(phi)*sin(theta))/(cos(theta)*cos(theta)), 0}};

  arma::mat::fixed<3,3> dfe_dw = R_b2e;

  arma::mat::fixed<3,3> dfw_dw = -arma::eye(3,3)*kr;

  arma::mat::fixed<3,3> dfp_de = {{ v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), w*cos(phi)*cos(psi)*cos(theta) - u*cos(psi)*sin(theta) + v*cos(psi)*cos(theta)*sin(phi), w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - u*cos(theta)*sin(psi)},
                                  {- v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), w*cos(phi)*cos(theta)*sin(psi) - u*sin(psi)*sin(theta) + v*cos(theta)*sin(phi)*sin(psi), w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta)},
                                  {                                                                v*cos(phi)*cos(theta) - w*cos(theta)*sin(phi),                          - u*cos(theta) - w*cos(phi)*sin(theta) - v*sin(phi)*sin(theta),                                                                                                                                   0}};
  
  arma::mat::fixed<3,3> dfp_dv  = Rr2n;

  arma::mat::fixed<3,3> dfv_dv = -arma::eye(3,3)*kp;


  Jfx.zeros();
  Jfx(arma::span(1,3),arma::span(1,3)) = dfe_de;
  Jfx(arma::span(1,3),arma::span(4,6)) = dfe_dw;
  Jfx(arma::span(4,6),arma::span(4,6)) = dfw_dw;
  Jfx(arma::span(7,9),arma::span(1,3)) = dfp_de;
  Jfx(arma::span(7,9),arma::span(10,12)) = dfp_dv;
  Jfx(arma::span(10,12),arma::span(10,12)) = dfv_dv;

  // JFu = [   0       0    ]
   //       [ duw_dsw   0    ]
   //       [   0       0    ]
   //       [   0    duv_dsv ]

  Jfu.zeros(); 
  Jfu(arma::span(4,6),arma::span(0,2)) = arma::eye(3,3);
  Jfu(arma::span(10,12),arma::span(3,5)) = arma::eye(3,3);
  

}

//---------------------------------------------------------------------------------------------------------

void Jac_quat_noise_driven_model2(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu)
{ 
  //
// Initial States ***************************************************
//  x meaning
//  index  1  2  3  4  5   6   7   8  9  10  11  12  13
//        q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z
//  Attitude states
//  x(1:4)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to body rotation)
//  x(5:7)=   [w_x w_y w_z ] ->  vel rotation in the navigation frame
//  Position states
//  x(8:10)= [x  y  z]  ->  Position in the navigation coordinate frame.
//  x(11:13)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.
// Arguments    : const double x[13]
//                double delta_t
//                const double k[2]
//                double JFx[169]
//                double JFu[78]
// Return Type  : void
// 
  // JFx = [ dfq_dq  dfq_dw   0       0    ]
  //       [   0     dfw_dw   0       0    ]
  //       [   0       0    dfp_dp  dfp_dv ]
  //       [   0       0      0     dfv_dv ]

  // JFu = [   0       0    ]
  //       [ duw_dsw   0    ]
  //       [   0       0    ]
  //       [   0    duv_dsv ]
  
   // Attitude equations
     double kr = 1/Tau_r;
    double kt = 1/Tau_p;   
     
    
    
    /// Attitude derivatives ---------------------------------------------------------------------------
    arma::vec w_u = x.subvec(4,6);
    arma::vec W = w_u*delta_t/2;
    double w = arma::norm(W);

    double sinwow;

    if (w==0)
    {
        sinwow = 1;
    }
    else
    {
        sinwow = sin(w)/w;
    }

    arma::mat W_mat = { {0, -W[0], -W[1], -W[2]},
                        {W[0], 0, -W[2], W[1]},
                        {W[1], W[2], 0, -W[0]},
                        {W[2], -W[1], W[0], 0} };    
    
    
    arma::mat::fixed<4,4> M = cos(w)*arma::eye(4,4) + W_mat*sinwow;

    // dfq_dq
    arma::mat::fixed<4,4> dfq_dq = M;    

    arma::vec::fixed<4> dCq_dcosw = x.subvec(0,3); // dCq_dcosw = [q1 q2 q3 q4]';
    double w_x = w_u(0);
    double w_y = w_u(1);
    double w_z = w_u(2);
    double dcosw_dnw = -sin(w);

    double e1 = pow(abs(delta_t*w_x),2/4) + pow(abs(delta_t*w_y),2/4) + pow(abs(delta_t*w_z),2/4) ;                                 
    arma::rowvec::fixed<3> dnw_dw = 
    {(delta_t*delta_t*w_x)/(4*pow(e1,1/2)), 
     (delta_t*delta_t*w_y)/(4*pow(e1,1/2)), 
     (delta_t*delta_t*w_z)/(4*pow(e1,1/2))};
    
    arma::mat::fixed<4,3> dC_dw = dCq_dcosw*dcosw_dnw*dnw_dw;
   
    arma::rowvec::fixed<3> dFs_dw;
   if (w==0)
    {
     dFs_dw = {0, 0, 0};
    }      
   else
    {     
     arma::rowvec::fixed<3> dFs_dab = { 1/w, -sin(w)/(w*w)};
     
     arma::vec::fixed<3> ab_dnw = {cos(w), 1};
     
     dFs_dw = dFs_dab*ab_dnw*dnw_dw;
    }
    double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    double q4 = x(3);
    arma::mat::fixed<4,3> dWq_dw = {{ -(delta_t*q2)/2, -(delta_t*q3)/2, -(delta_t*q4)/2},
                                    {  (delta_t*q1)/2,  (delta_t*q4)/2, -(delta_t*q3)/2},
                                    { -(delta_t*q4)/2,  (delta_t*q1)/2,  (delta_t*q2)/2},
                                    {  (delta_t*q3)/2, -(delta_t*q2)/2,  (delta_t*q1)/2}};

    arma::vec::fixed<4>  W_m = M*x.subvec(0,3);                                
    
    arma::mat::fixed<4,3> dB_dw = W_m*dFs_dw + sinwow*dWq_dw;
    
    // dfq_dw
    arma::mat::fixed<4,3> dfq_dw = dC_dw + dB_dw;
    
    // dfw_dw
    arma::mat::fixed<3,3> dfw_dw = (arma::eye(3,3)-arma::eye(3,3)*kr*delta_t);
    //arma::mat::fixed<3,3> dfw_dw = arma::eye(3,3);

    // Position derivatives ------------------------------------------------------------------------------------

    arma::mat::fixed<3,3> dfp_dp = arma::eye(3,3);
    arma::mat::fixed<3,3> dfp_dv = arma::eye(3,3)*delta_t;
    arma::mat::fixed<3,3> dfv_dv = (arma::eye(3,3)-arma::eye(3,3)*kt*delta_t);
    
  //  JFx = [ dfq_dq  dfq_dw   0       0    ]
  //        [   0     dfw_dw   0       0    ]
  //        [   0       0    dfp_dp  dfp_dv ]
  //        [   0       0      0     dfv_dv ]
    Jfx.zeros();
    Jfx(arma::span(0,3),arma::span(0,3)) = dfq_dq;
    Jfx(arma::span(0,3),arma::span(4,6)) = dfq_dw;
    Jfx(arma::span(4,6),arma::span(4,6)) = dfw_dw;
    Jfx(arma::span(7,9),arma::span(7,9)) = dfp_dp;
    Jfx(arma::span(7,9),arma::span(10,12)) = dfp_dv;
    Jfx(arma::span(10,12),arma::span(10,12)) = dfv_dv;
  
  
 // derivatives JFu
 //  JFu = [   0       0    ]
 //        [ duw_dsw   0    ]
//         [   0       0    ]
//         [   0    duv_dsv ]

  Jfu.zeros(); 
  Jfu(arma::span(4,6),arma::span(0,2)) = arma::eye(3,3);
  Jfu(arma::span(10,12),arma::span(3,5)) = arma::eye(3,3);
   
   //cout << dfq_dw << endl; 


}

//------------------------------------------------------------------------------------------------------------------
void Jac_euler_2_rotMat_transpose_by_P(arma::mat::fixed<3,3> &dfm_de, arma::vec::fixed<3> &p, double phi, double theta, double psi)
{
    double p_x = p(0);
    double p_y = p(1);
    double p_z = p(2);

   dfm_de = {{                                                                                                                                        0,                          - p_z*cos(theta) - p_x*cos(psi)*sin(theta) - p_y*sin(psi)*sin(theta),                                                                 p_y*cos(psi)*cos(theta) - p_x*cos(theta)*sin(psi)},
              {p_x*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - p_y*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + p_z*cos(phi)*cos(theta), p_x*cos(psi)*cos(theta)*sin(phi) - p_z*sin(phi)*sin(theta) + p_y*cos(theta)*sin(phi)*sin(psi), - p_x*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - p_y*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))},
              {p_x*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - p_y*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - p_z*cos(theta)*sin(phi), p_x*cos(phi)*cos(psi)*cos(theta) - p_z*cos(phi)*sin(theta) + p_y*cos(phi)*cos(theta)*sin(psi),   p_x*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + p_y*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))}};
  

}
//-----------------------------------------------------------------------------------------------------------
void Jac_euler_2_rotMat_by_P(arma::mat::fixed<3,3> &dfm_de, arma::vec::fixed<3> &p, double phi, double theta, double psi)
{
    double p_x = p(0);
    double p_y = p(1);
    double p_z = p(2);

    dfm_de = {{p_y*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + p_z*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), p_z*cos(phi)*cos(psi)*cos(theta) - p_x*cos(psi)*sin(theta) + p_y*cos(psi)*cos(theta)*sin(phi), p_z*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - p_y*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - p_x*cos(theta)*sin(psi)},
              {-p_y*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - p_z*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), p_z*cos(phi)*cos(theta)*sin(psi) - p_x*sin(psi)*sin(theta) + p_y*cos(theta)*sin(phi)*sin(psi), p_z*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - p_y*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + p_x*cos(psi)*cos(theta)},
              {                            p_y*cos(phi)*cos(theta) - p_z*cos(theta)*sin(phi),                          - p_x*cos(theta) - p_z*cos(phi)*sin(theta) - p_y*sin(phi)*sin(theta),                                                                                                                                         0}};

}

void Jac_e_init_feat_func(cv::Point2d uvd, arma::vec& x, CAM &camera_parameters, arma::mat  &Rc2r,arma::vec::fixed<3> &t_c2r, double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr)
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
   //           P_n = Rr2n*Rc2r*P_c +  (Rr2n*t_c2r + t_r2n)
   /*
    %dy_dx =  [ 0 dx_deuler     0(1x3)  1 0 0   0(1x3) ]
    %         [ 0 dy_deuler     0(1x3)  0 1 0   0(1x3) ]
    %         [ 0 dz_deuler     0(1x3)  0 0 1   0(1x3) ]

    %dy_dR =  [   dx_duv   dx_dr  ]
    %         [   dy_duv   dy_dr  ]
    %         [   dz_duv   dz_dr  ]
   */

    arma::vec::fixed<3> hc;
    arma::mat::fixed<3,2> dhc_duvd;    
 
    hc = Inverse_projection_model(uvd,1,true,camera_parameters,dhc_duvd); // compute inverse projection model
   
    double phi = x(1);
    double theta = x(2);
    double psi = x(3);
    double Ra2b[9];
    Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
    //arma::mat Rr2n(Ra2b,3,3); // robot to navigation rotation matrix
    arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix
    arma::mat::fixed<3,3> Rr2n = Rn2r.t(); 
   
    arma::mat Rc2n = Rr2n*Rc2r; 
        
    arma::vec::fixed<3> hn = Rc2n*hc;   //  

    arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature

    double hn1 = hn(0);
    double hn2 = hn(1);
    double hn3 = hn(2);
    
    double rhn = pow(hn1*hn1 + hn2*hn2 + hn3*hn3,1.5);

    arma::rowvec::fixed<3> dnormhn_dhn = { -hn1/rhn, -hn2/rhn, -hn3/rhn};

    arma::mat::fixed<3,3> dm_dhn = hn*dnormhn_dhn + arma::eye(3,3)*(1/arma::norm(hn));

    arma::mat::fixed<3,3> dP_dhn = depth*arma::eye(3,3)*dm_dhn;
    
    
    arma::vec::fixed<3> P_r = Rc2r*hc;
    
    arma::mat::fixed<3,3> dfp_de;
    Jac_euler_2_rotMat_transpose_by_P(dfp_de, P_r,phi, theta, psi);  

    arma::mat::fixed<3,3> dfp_tc_de;
    dfp_tc_de.set_size(3,3);
    Jac_euler_2_rotMat_transpose_by_P(dfp_tc_de, t_c2r,phi, theta, psi);    
      
    arma::mat::fixed<3,3> dP_deuler = dP_dhn*dfp_de  + dfp_tc_de; //---
    
    dy_dx.zeros();
    dy_dx(arma::span(0,2),arma::span(1,3)) = dP_deuler;
    dy_dx(arma::span(0,2),arma::span(7,9)) = arma::eye(3,3);

    //cout << dy_dx << endl;
    
    arma::mat::fixed<3,2> dP_duv = dP_dhn*Rc2n*dhc_duvd;
    dy_duvr(arma::span(0,2),arma::span(0,1)) = dP_duv;

    dy_duvr(arma::span(0,2),2) = m;

    //cout << dy_dx << endl; 
    //cout << dy_duvr << endl;

}

//-----------------------------------------------------------------------------------------------------------

void Jac_e_uv_XYZ(arma::vec::fixed<3> pxyz, arma::vec& x, CAM &camera_parameters, arma::mat &Rr2c,arma::vec::fixed<3> &t_c2r,arma::mat::fixed<2,13>& duv_dx, arma::mat::fixed<2,3>& duv_dy, bool Visual_update_attitude_update)
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

     double phi = x(1);
    double theta = x(2);
    double psi = x(3);
    double Ra2b[9];
    Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
    arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix       
    
    //arma::mat::fixed<3,3> Rn2r = Rr2n.t();
    //arma::mat::fixed<3,3> Rr2c = Rc2r.t();

    arma::mat::fixed<3,3> Rn2c = Rr2c*Rn2r;

    arma::vec::fixed<3> Tn2r = x.subvec(7,9);
   
   // pxyz = P^N  feature point location expressed in the navigation frame
   arma::vec::fixed<3> pnr = (pxyz - Tn2r);
   
   arma::vec::fixed<3> pc = Rn2c*pnr - Rr2c*t_c2r;

   arma::mat::fixed<2,3> duv_dPc;   

   cv::Point2d uvd = Projection_model(pc,1,true,camera_parameters,duv_dPc );

  
  
  arma::mat::fixed<3,3> dPc_de; 
  if(Visual_update_attitude_update == true)
  {

    arma::mat::fixed<3,3> dfp_de;
    Jac_euler_2_rotMat_by_P(dfp_de, pnr,phi, theta, psi); 
    //Jac_euler_2_rotMat_by_P(dfp_de, pnr,phi, theta, psi);   
    dPc_de = Rr2c*dfp_de;

  }
  else
  {
    dPc_de.zeros();
  }

  arma::mat::fixed<3,13> dPc_dx;
    dPc_dx.zeros();
    dPc_dx(arma::span(0,2),arma::span(1,3)) = dPc_de;
    dPc_dx(arma::span(0,2),arma::span(7,9)) = -1*Rn2c;

  //----------
    duv_dx = duv_dPc*dPc_dx;
  //-----------
  
    duv_dy = duv_dPc*Rn2c;

    //cout << duv_dx << endl;
    //cout << duv_dy << endl;


}

