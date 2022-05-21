#include "localslam_component.hpp"

namespace localslam
{


void EKFslam::setParameters()
{  
   //  Declare node parameters (and default values)
   this->declare_parameter<std::string>("Data_origin", "dataset");
   this->declare_parameter<double>("x_vel_run_time",1.0);
   this->declare_parameter<int>("Robot_state_size",13);
   this->declare_parameter<double>("Mono_cam_frame_period",0.0333);
   this->declare_parameter<std::string>("Prediction_model","noise_driven");
   this->declare_parameter<int>("Max_freq_output",100);
   this->declare_parameter<bool>("Attitude_update",true);
   this->declare_parameter<bool>("Altitude_update",true);
   this->declare_parameter<bool>("Speed_update",true);
   this->declare_parameter<bool>("Visual_update",true);
   this->declare_parameter<bool>("GPS_update",false);   
   this->declare_parameter<double>("Robot_init_x",0.0);
   this->declare_parameter<double>("Robot_init_y",0.0);
   this->declare_parameter<double>("Robot_init_z",0.0);
   this->declare_parameter<double>("Robot_init_roll",0.0);
   this->declare_parameter<double>("Robot_init_pitch",0.0);
   this->declare_parameter<double>("Robot_init_yaw",0.0);
   this->declare_parameter("Rot_spd_2_r", std::vector<double>{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});
   this->declare_parameter<double>("Range_sensor_offset",0.0);
   this->declare_parameter<double>("Range_sensor_max_range_pattern",6.0);
   this->declare_parameter<double>("Range_sensor_max_r_max",1.75);
   this->declare_parameter<double>("Range_sensor_max_range_operation",6.0);
   this->declare_parameter<double>("Sigma_w",0.001);   
   this->declare_parameter<double>("Sigma_a",1.0);
   this->declare_parameter<double>("Tau_r",1.0);
   this->declare_parameter<double>("Tau_p",100.0);
   this->declare_parameter<double>("Sigma_att_update",0.000000001);
   this->declare_parameter<double>("Sigma_spd_update",0.001);   
   this->declare_parameter<double>("Sigma_alt_update",0.1);
   this->declare_parameter<double>("Sigma_cam_uv",3.0);
   this->declare_parameter<double>("Sigma_cam_uv_anchors",5.0);
   this->declare_parameter<double>("Sigma_cam_initial_depth_range",0.1);
   this->declare_parameter<double>("Sigma_cam_initial_depth_WOrange",0.2);
   this->declare_parameter<int>("Alt_h_2_d",-1);
   this->declare_parameter<double>("Mono_cam_2_robot_axis_x",0.10);
   this->declare_parameter<double>("Mono_cam_2_robot_axis_y",0.0);
   this->declare_parameter<double>("Mono_cam_2_robot_axis_z",1.5707963268);
   this->declare_parameter<double>("Mono_cam_2_robot_pos_x",0.1);
   this->declare_parameter<double>("Mono_cam_2_robot_pos_y",0.0);
   this->declare_parameter<double>("Mono_cam_2_robot_pos_z",0.0);   
   this->declare_parameter<int>("Mono_cam_img_rows",240);
   this->declare_parameter<int>("Mono_cam_img_cols",320);
   this->declare_parameter("Mono_cam_distortions", std::vector<double>{0.02921, -0.00504, 0.00297, -0.00843, 0.00000});
   this->declare_parameter<double>("Mono_cam_cc_u",156.24435);
   this->declare_parameter<double>("Mono_cam_cc_v",117.04562);
   this->declare_parameter<double>("Mono_cam_fc_u",206.34225);
   this->declare_parameter<double>("Mono_cam_fc_v",268.65192);
   this->declare_parameter<double>("Mono_cam_alpha_c",0.0);  
   this->declare_parameter<int>("Images_minimun_distance_new_points",20);
   this->declare_parameter<int>("Images_half_patch_size_when_initialized",20);
   this->declare_parameter<int>("Images_half_patch_size_when_matching",6);
   this->declare_parameter<int>("Images_number_candidate_points_detected",20);
   this->declare_parameter<int>("Images_max_innov_pixels",50);   
   this->declare_parameter<int>("Visual_delete_Maximun_number_feats_allowed",100);
   this->declare_parameter<int>("Visual_delete_Maximun_number_anchors_allowed",100);
   this->declare_parameter<int>("Visual_delete_minimun_feats_in_state_before_delete",50);   
   this->declare_parameter<double>("Visual_delete_minimun_RatioMatched",0.3);   
   this->declare_parameter<bool>("Visual_update_use_1RANSAC_for_validation",false);
   this->declare_parameter<bool>("Visual_update_attitude_update",false);
   this->declare_parameter<double>("Visual_update_min_vel_for_update_feats",0.02);   
   this->declare_parameter<bool>("Visual_update_use_anchors",true);
   this->declare_parameter<double>("Visual_update_uncertainty_depth_anchor_init",0.07);
   this->declare_parameter<int>("Select_KeyFrames_min_n_frames_between_kf",5);
   this->declare_parameter<int>("Select_KeyFrames_min_n_matches",5);
   this->declare_parameter<double>("Select_KeyFrames_min_ratio_distance_depth",0.15);
   this->declare_parameter<bool>("GS_xy_update",true);
   this->declare_parameter<bool>("GS_z_update",true);
   this->declare_parameter<double>("GS_xy_update_max_delta",0.1);
   this->declare_parameter<double>("GS_z_update_max_delta",0.1);
   this->declare_parameter<bool>("CL_xy_update",true);
   this->declare_parameter<bool>("CL_z_update",true);
   this->declare_parameter<double>("CL_xy_update_max_delta",5.0);
   this->declare_parameter<double>("CL_z_update_max_delta",1.0);
   
      
  
   // Set parameter struct
   this->get_parameter("Data_origin", PAR.Data_origin);
   this->get_parameter("x_vel_run_time",PAR.x_vel_run_time);
   this->get_parameter("Robot_state_size",PAR.Robot_state_size);
   this->get_parameter("Mono_cam_frame_period",PAR.Mono_cam_frame_period);
   this->get_parameter("Prediction_model",PAR.Prediction_model);
   this->get_parameter("Max_freq_output",PAR.Max_freq_output);
   this->get_parameter("Attitude_update",PAR.Attitude_update);
   this->get_parameter("Altitude_update",PAR.Altitude_update);
   this->get_parameter("Speed_update",PAR.Speed_update);
   this->get_parameter("Visual_update",PAR.Visual_update);
   this->get_parameter("GPS_update",PAR.GPS_update);
   this->get_parameter("Robot_init_x",PAR.Robot_init_x);
   this->get_parameter("Robot_init_y",PAR.Robot_init_y);
   this->get_parameter("Robot_init_z",PAR.Robot_init_z);
   this->get_parameter("Robot_init_roll",PAR.Robot_init_roll);
   this->get_parameter("Robot_init_pitch",PAR.Robot_init_pitch);
   this->get_parameter("Robot_init_yaw",PAR.Robot_init_yaw);
    rclcpp::Parameter Rot_spd_2_r_par = this->get_parameter("Rot_spd_2_r");
    std::vector<double> Rot_spd_2_r_vec = Rot_spd_2_r_par.as_double_array();
    arma::mat::fixed<3,3> Rot_spd_2_r(&Rot_spd_2_r_vec.front());
    PAR.Rot_spd_2_r = Rot_spd_2_r;
   //cout << "Rot_spd_2_r: " << Rot_spd_2_r << endl; 
   this->get_parameter("Range_sensor_offset",PAR.Range_sensor_offset);
   this->get_parameter("Range_sensor_max_range_pattern",PAR.Range_sensor_max_range_pattern);
   this->get_parameter("Range_sensor_max_r_max",PAR.Range_sensor_max_r_max);
   this->get_parameter("Range_sensor_max_range_operation",PAR.Range_sensor_max_range_operation);
   this->get_parameter("Sigma_w",PAR.Sigma_w);   
   this->get_parameter("Sigma_a",PAR.Sigma_a);
   this->get_parameter("Tau_r",PAR.Tau_r);
   this->get_parameter("Tau_p",PAR.Tau_p);
   this->get_parameter("Sigma_att_update",PAR.Sigma_att_update);
   this->get_parameter("Sigma_spd_update",PAR.Sigma_spd_update);    
   this->get_parameter("Sigma_alt_update",PAR.Sigma_alt_update);
   this->get_parameter("Sigma_cam_uv",PAR.Sigma_cam_uv);
   this->get_parameter("Sigma_cam_uv_anchors",PAR.Sigma_cam_uv_anchors);
   this->get_parameter("Sigma_cam_initial_depth_range",PAR.Sigma_cam_initial_depth_range);
   this->get_parameter("Sigma_cam_initial_depth_WOrange",PAR.Sigma_cam_initial_depth_WOrange);
   this->get_parameter("Alt_h_2_d",PAR.Alt_h_2_d);
   this->get_parameter("Mono_cam_2_robot_axis_x",PAR.Mono_cam_2_robot_axis_x);
   this->get_parameter("Mono_cam_2_robot_axis_y",PAR.Mono_cam_2_robot_axis_y);
   this->get_parameter("Mono_cam_2_robot_axis_z",PAR.Mono_cam_2_robot_axis_z);
   this->get_parameter("Mono_cam_2_robot_pos_x",PAR.Mono_cam_2_robot_pos_x);
   this->get_parameter("Mono_cam_2_robot_pos_y",PAR.Mono_cam_2_robot_pos_y);
   this->get_parameter("Mono_cam_2_robot_pos_z",PAR.Mono_cam_2_robot_pos_z);
   this->get_parameter("Mono_cam_img_rows",PAR.Mono_cam_img_rows);
   this->get_parameter("Mono_cam_img_cols",PAR.Mono_cam_img_cols); 
    rclcpp::Parameter Mono_cam_distortions_par = this->get_parameter("Mono_cam_distortions");
    PAR.Mono_cam_distortions = Mono_cam_distortions_par.as_double_array();
   this->get_parameter("Mono_cam_cc_u",PAR.Mono_cam_cc_u);
   this->get_parameter("Mono_cam_cc_v",PAR.Mono_cam_cc_v);
   this->get_parameter("Mono_cam_fc_u",PAR.Mono_cam_fc_u);
   this->get_parameter("Mono_cam_fc_v",PAR.Mono_cam_fc_v);
   this->get_parameter("Mono_cam_alpha_c",PAR.Mono_cam_alpha_c);   
   this->get_parameter("Images_minimun_distance_new_points",PAR.Images_minimun_distance_new_points);
   this->get_parameter("Images_half_patch_size_when_initialized",PAR.Images_half_patch_size_when_initialized); 
   this->get_parameter("Images_half_patch_size_when_matching",PAR.Images_half_patch_size_when_matching);
   this->get_parameter("Images_number_candidate_points_detected",PAR.Images_number_candidate_points_detected);
   this->get_parameter("Images_max_innov_pixels",PAR.Images_max_innov_pixels);
   this->get_parameter("Visual_delete_Maximun_number_feats_allowed",PAR.Visual_delete_Maximun_number_feats_allowed); 
   this->get_parameter("Visual_delete_Maximun_number_anchors_allowed",PAR.Visual_delete_Maximun_number_anchors_allowed); 
   this->get_parameter("Visual_delete_minimun_feats_in_state_before_delete",PAR.Visual_delete_minimun_feats_in_state_before_delete);   
   this->get_parameter("Visual_delete_minimun_RatioMatched",PAR.Visual_delete_minimun_RatioMatched);   
   this->get_parameter("Visual_update_use_1RANSAC_for_validation",PAR.Visual_update_use_1RANSAC_for_validation);
   this->get_parameter("Visual_update_attitude_update",PAR.Visual_update_attitude_update);
   this->get_parameter("Visual_update_min_vel_for_update_feats",PAR.Visual_update_min_vel_for_update_feats);
   this->get_parameter("Visual_update_use_anchors",PAR.Visual_update_use_anchors);
   this->get_parameter("Visual_update_uncertainty_depth_anchor_init",PAR.Visual_update_uncertainty_depth_anchor_init);
   this->get_parameter("Select_KeyFrames_min_n_frames_between_kf",PAR.Select_KeyFrames_min_n_frames_between_kf);
   this->get_parameter("Select_KeyFrames_min_n_matches",PAR.Select_KeyFrames_min_n_matches);
   this->get_parameter("Select_KeyFrames_min_ratio_distance_depth",PAR.Select_KeyFrames_min_ratio_distance_depth);
   this->get_parameter("GS_xy_update",PAR.GS_xy_update);
   this->get_parameter("GS_z_update",PAR.GS_z_update);
   this->get_parameter("GS_xy_update_max_delta",PAR.GS_xy_update_max_delta);
   this->get_parameter("GS_z_update_max_delta",PAR.GS_z_update_max_delta);
   this->get_parameter("CL_xy_update",PAR.CL_xy_update);
   this->get_parameter("CL_z_update",PAR.CL_z_update);
   this->get_parameter("CL_xy_update_max_delta",PAR.CL_xy_update_max_delta);
   this->get_parameter("CL_z_update_max_delta",PAR.CL_z_update_max_delta);

       

}


}