#include "gmap.hpp"



//------------------------------------------------------------------
void GMAP::Update_Visibility_Graph_with_deleted_points(std::vector<std::vector<int>> &kf_vg_info)
{

    for(int i = 0; i < kf_vg_info.size(); i++ )
    {

        
        for(int j = 0; j < kf_vg_info[i].size(); j++ )
        { 
          for(int k = j+1 ; k < kf_vg_info[i].size(); k++)
          {
              int idx_kf_ref = kf_vg_info[i][j];

              int idx_kf_l = kf_vg_info[i][k];
            // cout << Gmap.Vgraph << endl;
              Gmap.Vgraph.at(idx_kf_ref,idx_kf_l)--;
              Gmap.Vgraph.at(idx_kf_l,idx_kf_ref)--;
            // cout << Gmap.Vgraph << endl;
          }  
        }

    }


}


//------------------------------------------------------------------
std::vector<int> GMAP::Get_idx_visually_linked_to_KF(int idx_KF)
{
    std::vector<int> idx_vl_kf;
    
    bool empty = true;
    
    for (int i = 0 ; i < idx_KF ; i++)
    {
      int n_vlp = (int)Gmap.Vgraph.at(idx_KF,i);
      if(n_vlp > 0)
      {
        idx_vl_kf.push_back(i);
        empty = false;
      }

    }
    
    if(empty == false)
    {
      return idx_vl_kf;
    }
    else
    {
      return std::vector<int>();
    }  
}



//----------------------------------------------------------------
void GMAP::Update_Visibility_Graph_with_new_KF()
{
  int idx_KF = Gmap.KeyFDATA.size()-1;

  int vg_size = Gmap.KeyFDATA.size();
  Gmap.Vgraph.resize(vg_size,vg_size); // increase size of visibility graph by one
  Gmap.Vgraph(vg_size-1,vg_size-1) = 0;

  arma::rowvec vg_row;
  vg_row.zeros(vg_size);

  
  // -----for taking into account the visual link with 0-kf
  if(idx_KF == 2)
  {
    for(int i = 0; i < Gmap.KeyFDATA[idx_KF-1].Idx_Matched_points.size();i++)
    {
      int64 idx_pt = Gmap.KeyFDATA[idx_KF-1].Idx_Matched_points[i];

      int idx_init_kf = Gmap.AnchorsDATA[idx_pt].init_KF;

      if  (idx_init_kf == 1)
      {
        Gmap.Vgraph.at(0,1)++;
        Gmap.Vgraph.at(1,0)++;
      }
    }
  }
  //----------------------------------------------------------

  for(int i = 0; i < Gmap.KeyFDATA[idx_KF].Idx_Matched_points.size();i++)
  {
      int64 idx_pt = Gmap.KeyFDATA[idx_KF].Idx_Matched_points[i];

      for (int j = 0; j < Gmap.KeyFDATA.size()-1 ; j++)
      {
        for(int k = 0; k < Gmap.KeyFDATA[j].Idx_Matched_points.size();k++)
        {
           if (idx_pt == Gmap.KeyFDATA[j].Idx_Matched_points[k])
           {
             vg_row[j]++;
             break;
           } 

        }

      }
      //int idx_init_kf = Gmap.AnchorsDATA[idx_pt].init_KF;

      //vg_row[idx_init_kf]++;

  }
  
  arma::vec vg_col = vg_row.t();      
  Gmap.Vgraph.row(vg_size-1)  = vg_row;
  Gmap.Vgraph.col(vg_size-1) = vg_col;




}

//--------------------------------------------------------------------------------
void GMAP::Update_Visibility_Graph_with_matches_of_new_points(std::vector<int64> &idx_new_points, std::vector<int> &idx_kf_m, std::vector<int64> &idx_pt_m )
{
    
    int n_new_points = idx_new_points.size();
    
    // add visual relation beetween the keyframes used for initialized the new points
    for (int i = 0; i < n_new_points; i++)
    {
      int idx_kf_c = Gmap.AnchorsDATA[idx_new_points[i]].init_KF;
      int idx_kf_p = idx_kf_c - 1;
      if (Gmap.AnchorsDATA[idx_new_points[i]].feat_type == "g_anchor")
        {
          Gmap.Vgraph.at(idx_kf_c,idx_kf_p)++;
          Gmap.Vgraph.at(idx_kf_p,idx_kf_c)++;
        
        }
    }


   //----------------------------------------------------------------------------
  
    for (int i = 0; i < idx_kf_m.size(); i++)
    {
        int idx_kf = idx_kf_m[i];
        int64 idx_pt = idx_pt_m[i];
        
        int idx_kf_pt = Gmap.AnchorsDATA[idx_pt].init_KF;
        
        //cout << Gmap.Vgraph << endl;
        Gmap.Vgraph.at(idx_kf,idx_kf_pt)++;
        Gmap.Vgraph.at(idx_kf_pt,idx_kf)++;
        //cout << Gmap.Vgraph << endl;
        int q = 10;

        if (Gmap.AnchorsDATA[idx_pt].feat_type == "g_anchor")
        {
          int idx_kf_prev_pt = idx_kf_pt-1;
          Gmap.Vgraph.at(idx_kf,idx_kf_prev_pt)++;
          Gmap.Vgraph.at(idx_kf_prev_pt,idx_kf)++;
          
        }




    }






}


//-------------------------------------------------------------------------------------
void  GMAP::Check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link )
{

    idx_direct_link = 0;
  if (Gmap.Vgraph.n_cols < 4)
  {
    idx_indirect_link = 0;
  }
  else
  { 
    //Vgraph.print();
    bool flag_zero = false;
    //int idx_i;
    for(int i = Gmap.Vgraph.n_cols-2; i >= 0 ;  i-- )
    {      
      int vlink =  Gmap.Vgraph(Gmap.Vgraph.n_rows-1,i);    
      if (vlink == 0)
      {
       flag_zero = true;
       idx_direct_link = i + 1;
       break;        
      }    
    } 
    if (flag_zero == false)
    {
      idx_indirect_link = 0;
    }
    else
    { 
      bool flag_zero2 = false;
      //int idx_j;
      for(int j = idx_direct_link-1; j >= 0 ; j--)
      {
        int vlink =  Gmap.Vgraph(idx_direct_link,j);
        if (vlink == 0)
          {
          flag_zero2 = true;
          idx_indirect_link  = j + 1;
          break;        
          }            
      }
      if (flag_zero2 == false)
      {
        idx_indirect_link = 0;
      }
      else
      { 
//        int q = 10;
        //return idx_j;
      }
        
    } // else 2
      
  } // else 1

}
//-----------------------------------------------------------------------------------------