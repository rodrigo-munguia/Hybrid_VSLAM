/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>
#include <armadillo>

#ifndef PARAMETERS_H
#define PARAMETERS_H

using namespace std;



struct parameters
{
    
    string Data_origin;
    double x_vel_run_time;


    
    int Robot_state_size;   
    
    // initial conditions
    double Robot_init_x;
    double Robot_init_y;
    double Robot_init_z;
    double Robot_init_roll;
    double Robot_init_pitch;
    double Robot_init_yaw;

    // prediction
    string Prediction_model;
    int Max_freq_output;    
    double Sigma_w;    
    double Sigma_a;        
    double Tau_r;
    double Tau_p;
    
    // Updates
    bool Attitude_update;
    bool Altitude_update;
    bool Speed_update;
    bool Visual_update;
    bool GPS_update;

    // attitude update
    double Sigma_att_update;
    // Speed update
    double Sigma_spd_update;
    arma::mat::fixed<3,3> Rot_spd_2_r;
    // Altitude update
    double Sigma_alt_update;
    int Alt_h_2_d;
    // Visual updates
    double Sigma_cam_uv;  // (pixels) camera measurement noise standard deviation (features)
    double Sigma_cam_uv_anchors;  // (pixels) camera measurement noise standard deviation (anchors)
    double Sigma_cam_initial_depth_range; // uncertanty (m) for initial hypotheses of depth for visual features with "range"
    double Sigma_cam_initial_depth_WOrange; // uncertanty (m) for initial hypotheses of depth for visual features without "range"

    // Range sensor
    double Range_sensor_offset; // offset for range measurements
    double Range_sensor_max_range_pattern; // max range of pattern, from sensor beam pattern
    double Range_sensor_max_r_max; // maximun radius of elipsoid at (max range of pattern) from sensor beam pattern
    double Range_sensor_max_range_operation; // max operation range of sensor 

    // Monocular camera parameters
    double Mono_cam_frame_period;  // 1/frame_rate  
    double Mono_cam_2_robot_axis_x;
    double Mono_cam_2_robot_axis_y;
    double Mono_cam_2_robot_axis_z;
    double Mono_cam_2_robot_pos_x;
    double Mono_cam_2_robot_pos_y;
    double Mono_cam_2_robot_pos_z;
    int Mono_cam_img_rows;
    int Mono_cam_img_cols;
    std::vector<double> Mono_cam_distortions;
    double Mono_cam_cc_u;
    double Mono_cam_cc_v;
    double Mono_cam_fc_u;
    double Mono_cam_fc_v;
    double Mono_cam_alpha_c;
    
    
    int Images_minimun_distance_new_points;
    int Images_half_patch_size_when_initialized;
    int Images_half_patch_size_when_matching;
    int Images_number_candidate_points_detected;
    int Images_max_innov_pixels;
    
    int Visual_delete_Maximun_number_feats_allowed;
    int Visual_delete_Maximun_number_anchors_allowed;
    int Visual_delete_minimun_feats_in_state_before_delete;    
    double Visual_delete_minimun_RatioMatched;
    
    
    
    bool Visual_update_use_1RANSAC_for_validation;
    double Visual_update_min_vel_for_update_feats;
    bool Visual_update_attitude_update;
    bool Visual_update_use_anchors;
    double Visual_update_uncertainty_depth_anchor_init;

    int Select_KeyFrames_min_n_frames_between_kf;
    int Select_KeyFrames_min_n_matches;
    double Select_KeyFrames_min_ratio_distance_depth;

    // position update from bundle adjustment (global slam)
    bool GS_xy_update;
    bool GS_z_update;
    double GS_xy_update_max_delta;
    double GS_z_update_max_delta;
    // position update from close loop (global slam)
    bool CL_xy_update;
    bool CL_z_update;
    double CL_xy_update_max_delta;
    double CL_z_update_max_delta;

} ;   





#endif /*PARAMETERS_H*/