/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>
#include <armadillo>

#ifndef PARAMETERS_GL_H
#define PARAMETERS_GL_H

using namespace std;



struct parameters
{
    
    // Monocular camera parameters    
    int Mono_cam_img_rows;
    int Mono_cam_img_cols;
    std::vector<double> Mono_cam_distortions;
    double Mono_cam_cc_u;
    double Mono_cam_cc_v;
    double Mono_cam_fc_u;
    double Mono_cam_fc_v;
    double Mono_cam_alpha_c;
    
    // visual match
    int VM_max_inov_error;

    // Initialization of global map points (anchors) 
    bool Init_use_anms_for_select_strong_points;  
    int Init_number_candidate_points_per_kf;
    int Init_min_distance_to_previous_mapped_features; // pixels
    double Init_min_depth_to_consider; // meters

    // Visibility Graph
    int VG_img_search_margin;
    
    // Delete function
    int Delete_min_kf_matched_for_keep_anchor;

    // Bundle adjusment
    int BA_max_n_kf_optimized;
    double BA_sigma_uv;
    double BA_sigma_kf_att;
    double BA_sigma_kf_xy;
    double BA_sigma_kf_z;
    double BA_sigma_kf_pt;
    bool BA_update_kf_pos;
    bool BA_update_kf_att;
    bool BA_update_map;



} ;   





#endif /*PARAMETERS_H*/