#ifndef EKF_H
#define EKF_H

//#include </usr/include/armadillo>
#include <armadillo>
#include "parameters.hpp"
#include "localslam_types.hpp"
#include "ekf_types.hpp"
#include <opencv2/opencv.hpp>
#include "../../globalslam/src/globalslam_types.hpp"
#include "../../common/Vision/vision.hpp"



using namespace arma;

class EKF
{

    public:        
                
                   
        
        EKF(parameters &par) // constructor
        {
            PAR = par;             
            initialized = false;                         
            
        }
        
         

    private:

        arma::vec x;  // State vector
        arma::mat P;  // Covariance Matrix 

        std::vector<FEAT> FeatsDATA;
        std::vector<FEAT> AnchorsDATA;

        CAM cam_parameters;
        arma::vec::fixed<3> t_c2r;  // vector defining the position of the camara respect to the robot frame 
        arma::mat::fixed<3,3> Rr2c; // Robot to camera rotation matrix

        KEYFRAME KF_selected; // for storing selected keyframes        
        KEYFRAME KF_cl; // for the closing loop process        
        bool NewKF_available;
        bool New_KF_cl;

        bool NewRobotState_available;        

        bool initialized; 
        parameters PAR;
        void prediction(double delta_t);
        void prediction_quat_noise_driven(double delta_t);
        void prediction_euler_differential_robot(ODOD &odod);
        void prediction_euler_odometry_vw_robot(ODOV &odov);
        void prediction_euler_noise_driven(double delta_t);
        void init_quat_system_state(DATA &dat);
        void init_euler_system_statet(DATA &dat);
        void Attitude_quat_Update(ATT &att);
        void Attitude_euler_Update(ATT &att);
        void Speed_Update2(SPD &spd);
        void Speed_Update_nav_euler(SPD &spd);
        void Altitude_update(ALT &alt);
        void Visual_update_e(FRAME *frame);
        void Visual_update_e_init_feat_wr(FRAME *frame);
        void Visual_update_e_init_feat_delayed(FRAME *frame);
        void Visual_update_e_init_anchors();
        void Get_img_points_for_init_feats(FRAME *frame, vector<cv::KeyPoint>& Points, cv::Mat &Descriptors, vector<cv::Mat> &Patches_init, vector<cv::Mat> &Patches_match);
        arma::vec::fixed<3> Triangulate_sigle_3d_point(cv::Point2f &uv1d,cv::Point2f &uv2d,arma::mat::fixed<3,3> &Rn2c_1,arma::vec::fixed<3> &t_c2n_1,arma::mat::fixed<3,3> &Rn2c_2,arma::vec::fixed<3> &t_c2n_2 );

        
        void Remove_close_points(vector<cv::KeyPoint>& output_KP, vector<cv::KeyPoint>&  input_KP);
        void Delete_i_feat_(int idx);
        int Visual_match_feats(FRAME *frame);
        int Visual_match_feats_by_descriptors_e(FRAME *frame);
        void Visual_match_SelectKeyFrames(FRAME *frame, int n_m_points,arma::vec::fixed<3> t_c2r, arma::mat::fixed<3,3> Rr2c, arma::vec::fixed<3> r_N, arma::mat::fixed<3,3> Rn2r);        
        void Visual_delete_feats();
        void Visual_update_WO_val();
        void Visual_update_With_val_e();
        void Visual_update_anchors_WO_val_e();
        void Visual_update_feats_WO_val_e();
        void Visual_update_One_Point_ransac_hypotheses_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a);
        void Visual_update_One_Point_ransac_e(std::vector<int> &z_lx_f,std::vector<int> &z_lx_a);
        void Visual_update_One_Point_ransac_rescue_hi_inliers_e(std::vector<int> &z_li_f,std::vector<int> &z_li_a,std::vector<int> &z_hi_f,std::vector<int> &z_hi_a );

        double yaw_at_home;
        


        
    
    public:
        void system_init(DATA &dat);
        void ekf_step(DATA &dat);       
        bool get_ekf_initialized_state();
        void set_ekf_initialized_statet(bool state); 
        LOCALSLAM_DATA get_lslam_data();
        bool get_KeyFrame(KEYFRAME &KF);
        bool get_KeyFrameCL(KEYFRAME &KF);
        bool get_RobotState(arma::vec::fixed<13> &x_r);
        void Update_pos_with_delta(arma::vec::fixed<3> delta_pos, string type);
        

    
};


#endif