#ifndef EKF_JAC_H
#define EKF_JAC_H

#include <cmath>
#include <cstring>
#include <armadillo>
#include "opencv2/opencv.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Vision/vision.hpp"




void Jac_quat_noise_driven_model2(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu);   
void Jac_euler_noise_driven_model(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu);

void Jac_euler_diff_driven_model(arma::vec& x, double delta_t, double R, double L,  arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,2>& Jfu);


void Jac_e_init_feat_func(cv::Point2d uvd, arma::vec& x, CAM &camera_parameters, arma::mat &Rc2r,arma::vec::fixed<3> &t_c2r, double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr);
void Jac_euler_2_rotMat_by_P(arma::mat::fixed<3,3> &dfm_de, arma::vec::fixed<3> &p, double phi, double theta, double psi);
void Jac_e_uv_XYZ(arma::vec::fixed<3> pxyz, arma::vec& x, CAM &camera_parameters, arma::mat &Rr2c,arma::vec::fixed<3> &t_c2r,arma::mat::fixed<2,13>& duv_dx, arma::mat::fixed<2,3>& duv_dy,bool Visual_update_attitude_update);



#endif