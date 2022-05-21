#include "globalslam_types.hpp"




void print_VG(GLOBAL_MAP &Gmap )
{
 cout << endl;
          for (int i = 0; i < Gmap.Vgraph.n_rows ; i++ )
          {
            for (int j = 0; j < Gmap.Vgraph.n_rows ; j++ )
            {
              cout << (int)Gmap.Vgraph.at(i,j) << " ";
            }
            cout << endl;    
          }
}          