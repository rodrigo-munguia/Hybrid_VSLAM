#include "cloop.hpp"


void CLOOP::Fuse_map_points(int &idx_kf_matched)
{
    int idx_new_kf = Gmap.KeyFDATA.size() -1;
    int idx_matched_kf = idx_kf_matched;    

    std::vector<int> idx_vl_new_kf = Get_n_visually_linked_kf(idx_new_kf ,10);
    idx_vl_new_kf.push_back(idx_new_kf);
    sort(idx_vl_new_kf.begin(), idx_vl_new_kf.end());

    std::vector<int> idx_vl_matched_kf = Get_n_visually_linked_kf(idx_matched_kf,10 );
    idx_vl_matched_kf.push_back(idx_matched_kf);
    sort(idx_vl_matched_kf.begin(), idx_vl_matched_kf.end());

    //-----------------------------------------------------------
    // get index of old points that can be potentially matched with recent points
    std::vector<int64> idx_old_points;
    for(int i = 0; i < idx_vl_matched_kf.size(); i++)
    {
        int idx_kf = idx_vl_matched_kf[i];

        for(int j = 0; j < Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size(); j++)
        {
            int64 idx_pt = Gmap.KeyFDATA[idx_kf].Idx_Matched_points[j];
            if ( std::find(idx_old_points.begin(), idx_old_points.end(), idx_pt) == idx_old_points.end() )
            {
                idx_old_points.push_back(idx_pt);
            }
        }     
    }
    //---------------------------------------------------------------
    // get index of recent points that can be potentially matched with old points
    std::vector<int64> idx_new_points;
    for(int i = 0; i < idx_vl_new_kf.size(); i++)
    {
        int idx_kf = idx_vl_new_kf[i];

        for(int j = 0; j < Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size(); j++)
        {
            int64 idx_pt = Gmap.KeyFDATA[idx_kf].Idx_Matched_points[j];
            if ( std::find(idx_new_points.begin(), idx_new_points.end(), idx_pt) == idx_new_points.end() )
            {
                idx_new_points.push_back(idx_pt);
            }
        }     
    }
    //------------------------------------------------------------------------
    std::vector<int64> idx_matched_new_pts;
    std::vector<int64> idx_matched_old_pts;
    Match_a_points_with_b_points(idx_new_points,idx_old_points,idx_matched_new_pts,idx_matched_old_pts);

    // idx_matched_new_pts contains the index of new points that have been matched with the corresponding index of contained in idx_matched_old_pts
    std::vector<int> idx_vl_kf_A;
    std::vector<int> idx_vl_kf_B;
    
    for (int i = 0; i < idx_matched_new_pts.size(); i ++)
    {
        int64 idx_new_pt = idx_matched_new_pts[i];
        int64 idx_old_pt = idx_matched_old_pts[i];
        
        // delete old point
        Gmap.AnchorsDATA[idx_old_pt].n_kf_matched += Gmap.AnchorsDATA[idx_new_pt].n_kf_matched; 
        Gmap.AnchorsDATA.erase(Gmap.AnchorsDATA.begin()+i);
        

        // get a list of kf where "idx_old_pt" has been matched
        std::vector<int64> idx_kf_old_pt;
        for (int j = 0; j < idx_vl_matched_kf[idx_vl_matched_kf.size()-1] ; j++)
        {
            for( int k = 0; k < Gmap.KeyFDATA[j].Idx_Matched_points.size(); k++)
            {
                if(Gmap.KeyFDATA[j].Idx_Matched_points[k] == idx_old_pt)
                {
                   idx_kf_old_pt.push_back(j); 
                }
            }    
        }         
        
        // replace index of matched new points with index of old matched points 
        for(int j = 0; j < idx_vl_new_kf.size() ; j++)
        {
            int idx_kf = idx_vl_new_kf[j];                        
            for( int k = 0; k < Gmap.KeyFDATA[idx_kf].Idx_Matched_points.size(); k++)
            {
                if(Gmap.KeyFDATA[idx_kf].Idx_Matched_points[k] == idx_new_pt)
                {
                     Gmap.KeyFDATA[idx_kf].Idx_Matched_points[k] = idx_old_pt;

                     // storing visually linked kf
                     for(int l = 0; l < idx_kf_old_pt.size();l++)
                     {   
                         int idx_kf_B = idx_kf_old_pt[l];
                         idx_vl_kf_A.push_back(idx_kf);
                         idx_vl_kf_B.push_back(idx_kf_B);
                     }   

                }
            }            
        }
       
    }
    
    // update visibility graph
    Update_Visibility_Graph_A_B_links(idx_vl_kf_A,idx_vl_kf_B);


    
    
    
    
    
    
    
    
    int q = 10;    

} 

