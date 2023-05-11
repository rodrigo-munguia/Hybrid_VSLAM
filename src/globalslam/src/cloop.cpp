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
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    
    static int n_kf_wo_cl = 0;
    // Get matches from old kf
    std::vector<int> idx_pt_matches;
    std::vector<cv::Point2d> image_points;
    int idx_kf_matched;
    
      auto ct_i = std::chrono::high_resolution_clock::now();  // take computation time

    Get_matches(kf_cl,idx_pt_matches,image_points,idx_kf_matched);
    n_kf_wo_cl++;

      auto ct_f = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
      double comp_search_time_per_step = elapsed.count() * 1e-9;
      total_comp_time = total_comp_time + comp_search_time_per_step;

    travel_dis += arma::as_scalar(arma::norm(kf_cl.t_c2n - last_pos));
    last_pos = kf_cl.t_c2n;        

    if (idx_pt_matches.size() > PAR.CL_min_n_matches && travel_dis > 5)
    { 
      /*
      cout << Gmap.Vgraph << endl;
      cout << "Points matched: " << endl;
      for(int k = 0; k < idx_pt_matches.size(); k++ ) cout << "Point: " << idx_pt_matches[k] << " init in kf: " << Gmap.AnchorsDATA[idx_pt_matches[k]].init_KF << endl;
      */
      bool get_pos;  
      arma::vec::fixed<3> kf_cl_pos;
      
      get_pos = Get_pos_of_current_frame(kf_cl_pos,kf_cl, idx_pt_matches, image_points,step);

      if(get_pos == true)
      {
        Update_gmap(kf_cl_pos,kf_cl,idx_kf_matched);
        cout << "Distance traveled since origin/last pos update: " << travel_dis << endl;
        
        if(PAR.Stats){
          store.cloop_stats.back().distance_traveled = travel_dis;
        }          
        
        travel_dis = 0;
        n_cloop_intents = 0;
        n_kf_wo_cl = 0;
        Close_loop = true;
      }

    }

    // store statitics 
     if(PAR.Stats){
          cout << "Cloop step:" << step << endl;
          store.total_comp_time = total_comp_time;
          store.time_search_per_step.first.push_back(step);
          store.time_search_per_step.second.push_back(comp_search_time_per_step);          
     }           




}