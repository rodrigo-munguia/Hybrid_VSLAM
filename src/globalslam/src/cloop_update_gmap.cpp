#include "cloop.hpp"


void CLOOP::Update_gmap(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl,int &idx_kf_matched)
{

     // Add the close loop frame as a new keyFrame in the global map
     Gmap.KeyFDATA.push_back(kf_cl); 
     //------------- Try to match the global map points in the new kf
     int idx_KF = Gmap.KeyFDATA.size()-1;  
     Match_Gmap_points_into_KF(idx_KF);            
     //------------- Update the visibility graph for taking into account the new keyframe
     Update_Visibility_Graph_with_new_KF(idx_KF);
       
    
     std::vector<arma::vec::fixed<3>> delta_kf_pos;
     std::vector<int> idx_kf_opt;

     Pose_SLAM(pos,delta_kf_pos,idx_kf_opt);
     
    // update kf position
    for(int i = 0 ; i < idx_kf_opt.size() ; i++)
    {   
        int idx_kf = idx_kf_opt[i];
        Gmap.KeyFDATA[idx_kf].t_c2n += delta_kf_pos[i]; 

    }
    // Update Point position
    for(int i = Gmap.AnchorsDATA.size()-1; i >= 0; i--)
    {
        int idx_init_kf = Gmap.AnchorsDATA[i].init_KF;

        auto it = find(idx_kf_opt.begin(), idx_kf_opt.end(), idx_init_kf);
                
        if(it != idx_kf_opt.end())
        {  
          int index = it - idx_kf_opt.begin();
           Gmap.AnchorsDATA[i].AnchorState += delta_kf_pos[index]; 
        }

    }

    
    
    Delta_kf_n = delta_kf_pos[delta_kf_pos.size()-1];

    // cout << "cloop: delta pos " << Delta_kf_n.t() << endl; 

    Gmap.idx_ref_pose_slam = Gmap.KeyFDATA.size()-2; // update index reference for the next close loop
    
    int q = 10;


    
    //---------- fuse the points for closing the loop
    
    // Fuse_map_points(idx_kf_matched);

    // ----------- try to get new matches
    std::vector<int> idx_vl_new_kf; 
    std::vector<int> idx_vl_old_kf;
    Update_gmap_with_new_matches(idx_kf_matched,idx_vl_new_kf,idx_vl_old_kf);
    
    
    // ----------- refine the global map after loop with (local) bundle adjustment
    Update_gmap_with_BA(idx_vl_new_kf, idx_vl_old_kf);
 






}  