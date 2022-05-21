#include "gmap.hpp"


//------------------------------------------
// Add a Keyframe to the buffer
// Rodrigo M. 20222
void GMAP::Add_Kf(KEYFRAME &kf)
{

    Kf_buffer.push_back(kf);
    //cout << "Kf " << Kf_buffer.size() << " added" << endl;

}

//----------------------------------------
// Get Global Map
void GMAP::Get_GlobalMap(GLOBAL_MAP &Gmap_c)
{
   Gmap_c = Gmap;
}
//------------------------------------------
// Set global map
// Rodrigo M. 20222
void GMAP::Set_GlobalMap(GLOBAL_MAP &Gmap_c)
{
  Gmap = Gmap_c;
}

//------------------------------------------
arma::vec::fixed<3>  GMAP::Get_delta_pos()
{
  return Delta_kf_n;      
}



//------------------------------------------
// Perform a full global map update step
// Rodrigo M. 20222
void GMAP::Update()
{
    
    //------------------------------------------------------
    // After a close loop perform a full update of the visibility graph
    
    
    //-------------------------------------------------------
    // Grow the global map: Add new Keyframes and new Map Point
    bool new_kf = false;
    for(int k = 0; k < Kf_buffer.size() ; k++ )
    {   
        // Add new keyframes to the global map
        Gmap.KeyFDATA.push_back(Kf_buffer[k]);
        new_kf = true; // if a new keyframe has been added set this flag
        //cout << Kf_buffer[k].frame << endl;
        
            //------------- Try to match the global map points in the new kf  
            Match_Gmap_points_into_new_KF();
            
            //------------- Update the visibility graph for taking into account the new keyframe
            
            Update_Visibility_Graph_with_new_KF();
           
         
        if (Gmap.KeyFDATA.size() > 1)        
        { 
            //------------- Initialize new global map points
            std::vector<int64> idx_new_points = Init_new_map_points();
            //cout << idx_new_points.size() << endl;
            //cout << Gmap.Vgraph << endl;
            
            if (idx_new_points.size() > 0) // if new map points have been initialized
            {          
                std::vector<int> idx_kf_m; 
                std::vector<int64> idx_pt_m; 
                //-------------- Try to match the new map points in previous KF
                Match_New_points_into_previous_KF(idx_new_points, idx_kf_m, idx_pt_m );                

                //-------------- Update the visibility graph for taking into account the new map points
                //cout << " Before: " << endl;
                //cout << Gmap.Vgraph << endl;
                Update_Visibility_Graph_with_matches_of_new_points(idx_new_points,idx_kf_m, idx_pt_m );
                // cout << " After: " << endl;
                //cout << Gmap.Vgraph << endl;
                //( solve first row/column of zeros)

                //cout << Gmap.Vgraph << endl;       
            }
        }
        
    }
    Kf_buffer.clear();
    //--------------------------------------------------------

   if ((new_kf == true)&&(Gmap.KeyFDATA.size()>2))
        {
           // Delete map points that have not been matched in a minimun number of keyframes
           std::vector<std::vector<int>> kf_vg_info; 
           Delete_points(kf_vg_info);
           
           // Update visibility graph           
           Update_Visibility_Graph_with_deleted_points(kf_vg_info);
           
           //cout << Gmap.Vgraph << endl;
           
           // perform (local) bundle adjustment over the global map
           Bundle_adjustment(); 

           // Update matches and visibility graph  
           /*
           cout << endl;
           for (int i = 0; i < Gmap.Vgraph.n_rows ; i++ )
            {
                for (int j = 0; j < Gmap.Vgraph.n_rows ; j++ )
                    {
                        cout << (int)Gmap.Vgraph.at(i,j) << " ";
                    }
                    cout << endl;    
                }
           */     
           int q = 10;
        }



}