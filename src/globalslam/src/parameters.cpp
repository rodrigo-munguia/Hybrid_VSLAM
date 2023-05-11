#include "globalslam_component.hpp"

namespace globalslam
{


void Gslam::setParameters()
{  
   //  Declare node parameters (and default values)
   this->declare_parameter<int>("Mono_cam_img_rows",240);
   this->declare_parameter<int>("Mono_cam_img_cols",320);
   this->declare_parameter("Mono_cam_distortions", std::vector<double>{0.02921, -0.00504, 0.00297, -0.00843, 0.00000});
   this->declare_parameter<double>("Mono_cam_cc_u",156.24435);
   this->declare_parameter<double>("Mono_cam_cc_v",117.04562);
   this->declare_parameter<double>("Mono_cam_fc_u",206.34225);
   this->declare_parameter<double>("Mono_cam_fc_v",268.65192);
   this->declare_parameter<double>("Mono_cam_alpha_c",0.0);
   this->declare_parameter<int>("Init_number_candidate_points_per_kf",500);
   this->declare_parameter<bool>("Init_use_anms_for_select_strong_points",true);
   this->declare_parameter<double>("Init_anms_tolerance",0.3);
   this->declare_parameter<int>("Init_min_distance_to_previous_mapped_features",5);
   this->declare_parameter<double>("Init_min_depth_to_consider",0.2);
   this->declare_parameter<int>("VM_max_inov_error",20);
   this->declare_parameter<int>("VG_img_search_margin",15);
   this->declare_parameter<int>("Delete_min_kf_matched_for_keep_anchor",3);
   this->declare_parameter<int>("BA_max_n_previous_kf_for_search_matches",20);
   this->declare_parameter<int>("BA_max_n_kf_optimized",7);
   this->declare_parameter<int>("BA_min_n_latest_kf_to_optimize",4);
   this->declare_parameter<double>("BA_sigma_uv",1.0);
   this->declare_parameter<double>("BA_sigma_kf_att",0.01);
   this->declare_parameter<double>("BA_sigma_kf_xy",0.1); 
   this->declare_parameter<double>("BA_sigma_kf_z",0.1);
   this->declare_parameter<double>("BA_sigma_kf_pt",0.2);
   this->declare_parameter<bool>("BA_update_kf_pos",true);
   this->declare_parameter<bool>("BA_update_kf_att",true);
   this->declare_parameter<bool>("BA_update_map",true);
   this->declare_parameter<double>("BA_max_delta_kf_pos",0.2);
   this->declare_parameter<int>("CL_min_n_not_vl_kf",2);
   this->declare_parameter<int>("CL_min_n_matches",10);
   this->declare_parameter<double>("CL_min_mean_reprojection_error",25.0);
   this->declare_parameter<double>("CL_xy_update_max_delta",6.0);
   this->declare_parameter<double>("CL_odo_sigma_kf_att",0.0001);
   this->declare_parameter<double>("CL_odo_sigma_kf_xy",0.05);
   this->declare_parameter<double>("CL_odo_sigma_kf_z",0.05);
   this->declare_parameter<double>("CL_clo_sigma_kf_att",0.00001);
   this->declare_parameter<double>("CL_clo_sigma_kf_xy",0.005);
   this->declare_parameter<double>("CL_clo_sigma_kf_z",0.005);
   this->declare_parameter<bool>("CL_update_z",true);
   this->declare_parameter<bool>("Stats",false);
   
      
  
   // Set parameter struct
   //this->get_parameter("Data_origin", PAR.Data_origin);
   this->get_parameter("Mono_cam_img_rows",PAR.Mono_cam_img_rows);
   this->get_parameter("Mono_cam_img_cols",PAR.Mono_cam_img_cols); 
    rclcpp::Parameter Mono_cam_distortions_par = this->get_parameter("Mono_cam_distortions");
    PAR.Mono_cam_distortions = Mono_cam_distortions_par.as_double_array();
   this->get_parameter("Mono_cam_cc_u",PAR.Mono_cam_cc_u);
   this->get_parameter("Mono_cam_cc_v",PAR.Mono_cam_cc_v);
   this->get_parameter("Mono_cam_fc_u",PAR.Mono_cam_fc_u);
   this->get_parameter("Mono_cam_fc_v",PAR.Mono_cam_fc_v);
   this->get_parameter("Mono_cam_alpha_c",PAR.Mono_cam_alpha_c);
   this->get_parameter("Init_number_candidate_points_per_kf",PAR.Init_number_candidate_points_per_kf);
   this->get_parameter("Init_use_anms_for_select_strong_points",PAR.Init_use_anms_for_select_strong_points);
   this->get_parameter("Init_anms_tolerance",PAR.Init_anms_tolerance);
   this->get_parameter("Init_min_distance_to_previous_mapped_features",PAR.Init_min_distance_to_previous_mapped_features);
   this->get_parameter("Init_min_depth_to_consider",PAR.Init_min_depth_to_consider);
   this->get_parameter("VM_max_inov_error",PAR.VM_max_inov_error);
   this->get_parameter("VG_img_search_margin",PAR.VG_img_search_margin);
   this->get_parameter("Delete_min_kf_matched_for_keep_anchor",PAR.Delete_min_kf_matched_for_keep_anchor);
   this->get_parameter("BA_max_n_previous_kf_for_search_matches",PAR.BA_max_n_previous_kf_for_search_matches);
   this->get_parameter("BA_max_n_kf_optimized",PAR.BA_max_n_kf_optimized);
   this->get_parameter("BA_min_n_latest_kf_to_optimize",PAR.BA_min_n_latest_kf_to_optimize);
   this->get_parameter("BA_sigma_uv",PAR.BA_sigma_uv);    
   this->get_parameter("BA_sigma_kf_att",PAR.BA_sigma_kf_att);
   this->get_parameter("BA_sigma_kf_xy",PAR.BA_sigma_kf_xy);
   this->get_parameter("BA_sigma_kf_z",PAR.BA_sigma_kf_z);
   this->get_parameter("BA_sigma_kf_pt",PAR.BA_sigma_kf_pt);
   this->get_parameter("BA_update_kf_pos",PAR.BA_update_kf_pos);
   this->get_parameter("BA_update_kf_att",PAR.BA_update_kf_att);
   this->get_parameter("BA_update_map",PAR.BA_update_map);
   this->get_parameter("BA_max_delta_kf_pos",PAR.BA_max_delta_kf_pos);
   this->get_parameter("CL_min_n_not_vl_kf",PAR.CL_min_n_not_vl_kf);
   this->get_parameter("CL_min_n_matches",PAR.CL_min_n_matches);
   this->get_parameter("CL_min_mean_reprojection_error",PAR.CL_min_mean_reprojection_error);
   this->get_parameter("CL_xy_update_max_delta",PAR.CL_xy_update_max_delta);
   this->get_parameter("CL_odo_sigma_kf_att",PAR.CL_odo_sigma_kf_att);
   this->get_parameter("CL_odo_sigma_kf_xy",PAR.CL_odo_sigma_kf_xy);
   this->get_parameter("CL_odo_sigma_kf_z",PAR.CL_odo_sigma_kf_z);
   this->get_parameter("CL_clo_sigma_kf_att",PAR.CL_clo_sigma_kf_att);
   this->get_parameter("CL_clo_sigma_kf_xy",PAR.CL_clo_sigma_kf_xy);
   this->get_parameter("CL_clo_sigma_kf_z",PAR.CL_clo_sigma_kf_z);
   this->get_parameter("CL_update_z",PAR.CL_update_z);
   this->get_parameter("Stats",PAR.Stats);

  
  
   
  
  
   
   


}


}