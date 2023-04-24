#include "getData.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include <armadillo>
#include <filesystem>
#include <iostream>


/*---------------------------------------------------
Rodrigo Munguía 2022.

Get data from Lori dataset: https://shimo.im/docs/HhJj6XHYhdRQ6jjk/read
-----------------------------------------------------
*/

using namespace std;
using namespace cv;


string Odometry_dat = "odom.txt";
string Cam_dir = "color/";
string Cam_list = "color.txt";


//-------------------------------------------------------------------------------------------------
// get data from Lori dataset
DATA getDataL(parameters &PAR)
{

 string Dataset_dir = PAR.Dataset_path;
 
 DATA dat;
  
    static int flag_init = 0;
    static vector<FRAME> dat_F;
    static vector<ODOV> dat_V; 
    
    static auto last_time_data = std::chrono::high_resolution_clock::now();
    static double last_t = 10000000000000000;
    
    if (PAR.restart == true)
    {      
       flag_init = 0;
       dat_F.clear();
       dat_V.clear(); 
       
       last_time_data = std::chrono::high_resolution_clock::now();
       last_t = 10000000000000000;
       PAR.restart = false; 
    }


    
    // load data from the text files ----------
    // Execute only one time
    if (flag_init == 0)
    {   
        ifstream data_file_D;
        ifstream data_file_F;
            

        data_file_D.open(Dataset_dir + Odometry_dat); // frames file
        data_file_F.open(Dataset_dir + Cam_list);
        

        string lineV;
        vector<string> v;
        getline(data_file_D,lineV); // for "jump" the header
        while ( getline(data_file_D,lineV) )
        {  
            stringstream ss(lineV); 
            ODOV odo;
            vector<string> v_V;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ' ' );
                v_V.push_back( substr );
            }
           // gps.time = stoi(v_G[0]);
            long int t_f = (stod(v_V[0])*1000000);
            odo.time = (long int)t_f%10000000000;
            odo.linear_vel = stod(v_V[8]);
            odo.angular_vel = stod(v_V[13]);
            if(odo.time  > PAR.init_time) dat_V.push_back(odo);    
        }
        

        string lineF;
       
        while ( getline(data_file_F,lineF) )
        {  
           stringstream ss(lineF);
           FRAME frame;
           vector<string> v_F;
           
           while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ' ' );
                v_F.push_back( substr );
            }
                    
           long int t_f = (stod(v_F[0])*1000000);
           frame.time = (long int)t_f%10000000000;   
          
           frame.image_file = v_F[1];
            //int q = 10;
           if(frame.time  > PAR.init_time) dat_F.push_back(frame);     


        }

        flag_init = 1;
        data_file_D.close();
        data_file_F.close();


    }

    //---------------------------------------------- 
    

    int idx;
    double current_t; 
    
    long int times[2] = {dat_F[0].time,dat_V[0].time};
    idx = distance(begin(times),min_element(begin(times), end(times)));
    current_t = times[idx]; 
    

    if ((dat_F.size() == 0) || (dat_V.size() == 0) )
    {  
     //cout << "dataset: no more data" << endl;
     dat.data_type = "NULL";
     return dat;
    } 

        
        // compute actual time elapsed from the last time a data was query
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - last_time_data); 
        double delta_t = elapsed.count() * 1e-9; 
        //
        double elapsed_ds = (current_t - last_t)/1000000;
        //double elapsed_ds = (current_t - last_t);

        

        //cout << elapsed_ds << endl;
        
        dat.data_type = "";         

        if(delta_t*PAR.x_vel_run_time > elapsed_ds)
        { 
                //cout << delta_t << endl;
                if (idx == 0)
                {   
                    
                    dat.data_type = "frame";
                    dat.time = dat_F[0].time;
                    
                    dat.frame = dat_F[0];
                    // read image:  it can be move up to the parse section
                    cv::Mat image;
                    image = imread(Dataset_dir + dat_F[0].image_file, IMREAD_GRAYSCALE  );

                     Mat image_down;
                    cv::resize(image, image_down, Size(424, 240), INTER_LINEAR);

                    dat.frame.image = image_down;
                    dat.frame.range = -1;                            
                    
                    dat_F.erase(dat_F.begin());
                    
                }
                else if(idx == 1)
                {   
                    dat.data_type = "odov";
                    dat.time = dat_V[0].time;
                    dat.odov = dat_V[0];
                    dat_V.erase(dat_V.begin());
                }                
                else
                {
                    
                }
                last_t = dat.time; // dataset time 
                last_time_data = std::chrono::high_resolution_clock::now();  // actual time 

                   
        }

 
    
    return dat;
}
