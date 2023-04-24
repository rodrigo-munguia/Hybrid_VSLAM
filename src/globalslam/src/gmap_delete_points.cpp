

#include "gmap.hpp"


///------------------------------------------------------------------------------
void GMAP::Delete_points(std::vector<std::vector<int>> &kf_vg_info)
{
  
  //cout << Gmap.AnchorsDATA.size() << endl;

  for (int64 i = Gmap.AnchorsDATA.size()-1; i >=0 ; i--)
  {
      
      int &n_matches = Gmap.AnchorsDATA[i].n_kf_matched;
      int &n_m_tries = Gmap.AnchorsDATA[i].n_tries_matchs;

      if(n_m_tries > (PAR.Delete_min_kf_matched_for_keep_anchor-3) && n_matches < PAR.Delete_min_kf_matched_for_keep_anchor )
      {
          
          // remove from Gmap.KeyFDATA (consider to improve this code to not consider every kf)
          int n_del = 0;
          
          std::vector<int> kf_vl; 

          for (int j = Gmap.KeyFDATA.size()-1; j >=0 ; j--)
          { 
            //auto it = std::find(Gmap.KeyFDATA[j].Idx_Matched_points.begin(), Gmap.KeyFDATA[j].Idx_Matched_points.end(), i);
                    
              for (int64 k = 0 ; k < Gmap.KeyFDATA[j].Idx_Matched_points.size(); k++)
              {
                if(Gmap.KeyFDATA[j].Idx_Matched_points[k] == i )
                {                   
                  Gmap.KeyFDATA[j].Idx_Matched_points.erase(Gmap.KeyFDATA[j].Idx_Matched_points.begin()+k);
                  Gmap.KeyFDATA[j].UV_Matched_points.erase(Gmap.KeyFDATA[j].UV_Matched_points.begin()+k);
                  kf_vl.push_back(j);
                  n_del++;
                 // cout << "Point: " << i << " init in kf: " << Gmap.AnchorsDATA[i].init_KF << " deleted from kf: " << j << endl;
                }
                if(Gmap.KeyFDATA[j].Idx_Matched_points[k] > i )
                {
                  Gmap.KeyFDATA[j].Idx_Matched_points[k]--;
                }  
              }
              //for_each(Gmap.KeyFDATA[j].Idx_Matched_points.begin() + idx_pt_in_kf , Gmap.KeyFDATA[j].Idx_Matched_points.end(), [value](auto& x) { x -= value; });
              //std::replace_if(Gmap.KeyFDATA[j].Idx_Matched_points.begin(), Gmap.KeyFDATA[j].Idx_Matched_points.end(), [&i](int value) {return value > i;}, [](int value){return value-1;});
              //for(int k = 0; k < Gmap.KeyFDATA.size() ; k++)
              //  for(auto& value :Gmap.KeyFDATA[j].Idx_Matched_points )
              //  { 
              //    if(value > i) value = value -1;
              //  }

              
            //}
          
          }

          kf_vg_info.push_back(kf_vl);
          
          /*
          if(n_del > 2)
          { 
         //   cout << Gmap.AnchorsDATA[i].n_kf_matched << endl;
            int q = 10;
          }
          */
          Gmap.AnchorsDATA.erase(Gmap.AnchorsDATA.begin()+i);
         
          
      }
  }




}


