#ifndef GLOOP_H
#define GLOOP_H

#include <armadillo>
#include "parameters.hpp"
#include "../../common/Vision/vision.hpp"
#include "globalslam_types.hpp"
#include <opencv2/opencv.hpp>

class CLOOP
{

    public:        
                
                   
        
        CLOOP(parameters &par) // constructor
        {
           PAR = par;

           cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
           cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
           cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
           cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
           cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
           cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;

            Close_loop = false;
        }
        
         

    private:

        GLOBAL_MAP Gmap; // Global map structure

        parameters PAR; // parameters structure
        
        CAM cam_parameters;

        arma::vec::fixed<3> Delta_kf_n;

        bool Close_loop;

        std::vector<int> Get_n_not_visually_linked_kf(int idx_kf , int idx_kf_sup_limit );
        std::vector<int> Get_n_visually_linked_kf(int idx_kf, int min_strenght);
       
        void Get_matches(KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points ,int &idx_kf_matched);
        
        bool Get_pos_of_current_frame(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points);

        void Update_gmap(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl,int &idx_kf_matched);
        
        void Pose_SLAM(arma::vec::fixed<3> &pos, std::vector<arma::vec::fixed<3>> &delta_kf_pos, std::vector<int> &idx_kf_opt);
        
        void Match_Gmap_points_into_KF(int idx_KF);
        void Update_Visibility_Graph_with_new_KF(int idx_KF);

        void Update_Visibility_Graph_A_B_links(std::vector<int> &idx_vl_kf_A,std::vector<int> &idx_vl_kf_B);

        void Fuse_map_points(int &idx_kf_matched);

        void Update_gmap_with_new_matches(int &idx_kf_matched, std::vector<int> &idx_vl_new_kf, std::vector<int> &idx_vl_old_kf);        
        void Match_list_of_points_into_KF(std::vector<int64> &idx_points, int idx_KF, std::vector<int64> &idx_matched_points);
        void Update_Visibility_Graph_with_matched_points_into_KF(std::vector<int64> &idx_points, int idx_KF);

        void Match_a_points_with_b_points(std::vector<int64> &idx_a_points,std::vector<int64> &idx_b_points,std::vector<int64> &idx_matched_a_pts,std::vector<int64> &idx_matched_b_pts);    

        void Update_gmap_with_BA(std::vector<int> &idx_vl_new_kf, std::vector<int> &idx_vl_old_kf); 

    public:
        
        void Set_GlobalMap(GLOBAL_MAP &Gmap_c);

        void Get_GlobalMap(GLOBAL_MAP &Gmap_c);

        void Step(KEYFRAME &kf_cl);

        arma::vec::fixed<3>  Get_delta_pos();

        bool Get_close_loop_state();
        void Set_close_loop_state(bool state);

        

    
};


#endif


