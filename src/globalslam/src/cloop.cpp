#include "cloop.hpp"



//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Set_GlobalMap(GLOBAL_MAP &Gmap_c)
{

  Gmap = Gmap_c;

}
//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Get_GlobalMap(GLOBAL_MAP &Gmap_c)
{
  Gmap_c = Gmap;
}

//------------------------------------------
arma::vec::fixed<3>  CLOOP::Get_delta_pos()
{
  return Delta_kf_n;      
}
//----------------------------------------
bool CLOOP::Get_close_loop_state()
{
  return Close_loop;
}
void CLOOP::Set_close_loop_state(bool state)
{
  Close_loop = state;
}


//------------------------------------------
// Perform a full step for trying to close a loop
// Rodrigo M. 20222
void CLOOP::Step(KEYFRAME &kf_cl)
{
    static int n_kf_wo_cl = 0;
    // Get matches from old kf
    std::vector<int> idx_pt_matches;
    std::vector<cv::Point2d> image_points;
    int idx_kf_matched;
    
    Get_matches(kf_cl,idx_pt_matches,image_points,idx_kf_matched);
    n_kf_wo_cl++;

    if (idx_pt_matches.size() > PAR.CL_min_n_matches && n_kf_wo_cl > PAR.CL_min_n_not_vl_kf)
    { 
      /*
      cout << Gmap.Vgraph << endl;
      cout << "Points matched: " << endl;
      for(int k = 0; k < idx_pt_matches.size(); k++ ) cout << "Point: " << idx_pt_matches[k] << " init in kf: " << Gmap.AnchorsDATA[idx_pt_matches[k]].init_KF << endl;
      */
      bool get_pos;  
      arma::vec::fixed<3> kf_cl_pos;
      
      get_pos = Get_pos_of_current_frame(kf_cl_pos,kf_cl, idx_pt_matches, image_points);

      if(get_pos == true)
      {
        Update_gmap(kf_cl_pos,kf_cl,idx_kf_matched);

        n_kf_wo_cl = 0;
        Close_loop = true;
      }

    }


}