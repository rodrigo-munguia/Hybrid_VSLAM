#include "getData.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include <armadillo>
#include <filesystem>
#include <iostream>


/*---------------------------------------------------
Rodrigo Munguía 2022.

Get data from Rawseeds data base
-----------------------------------------------------
*/

using namespace std;
using namespace cv;

string Odometry_data = "Bicocca_2009-02-25b-ODOMETRY_XYT.csv";
//string Omni_dir = "OMNI/";
//string Omni_list = "Bicocca_2009-02-25b-LISTS/Bicocca_2009-02-25b-OMNI.csv";

string Frontal_dir = "FRONTAL/";
string Frontal_list = "Bicocca_2009-02-25b-LISTS/Bicocca_2009-02-25b-FRONTAL.csv";


//-------------------------------------------------------------------------------------------------
// get data from Rawseeds dataset
DATA getDataR(parameters &PAR)
{

 string Dataset_dir = PAR.Dataset_path;
 
 DATA dat;
  
    static int flag_init = 0;
    static vector<FRAME> dat_F;
    static vector<ODOD> dat_D; 
    
    static auto last_time_data = std::chrono::high_resolution_clock::now();
    static double last_t = 10000000000000000;
    
    if (PAR.restart == true)
    {      
       flag_init = 0;
       dat_F.clear();
       dat_D.clear(); 
       
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
            

        data_file_D.open(Dataset_dir + Odometry_data); // frames file
        data_file_F.open(Dataset_dir + Frontal_list);
        

        string lineD;
        vector<string> v;
        while ( getline(data_file_D,lineD) )
        {  
            stringstream ss(lineD); 
            ODOD odo;
            vector<string> v_D;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_D.push_back( substr );
            }
           // gps.time = stoi(v_G[0]);
            double t_f = (stod(v_D[0])*1000000);
            odo.time = (long int)t_f%10000000000;
            odo.TicksRight = stod(v_D[2]);
            odo.TicksLeft = stod(v_D[3]);
            if(odo.time  > PAR.init_time) dat_D.push_back(odo);    
        }
        

        string lineF;
       
        while ( getline(data_file_F,lineF) )
        {  

           FRAME frame;
           // frame.time = stoi(lineF.substr(0,10));
           
           double t_f = (stod(lineF)*1000000);
           frame.time = (long int)t_f%10000000000;    
          
           frame.image_file = "FRONTAL_"+ lineF + ".png";
            //int q = 10;
           if(frame.time  > PAR.init_time) dat_F.push_back(frame);     


        }
        /*  
        string path = Dataset_dir + Omni_dir;

        for (const auto & entry : std::filesystem::directory_iterator(path))
        {   
            FRAME frame;
            string img_file = entry.path();            
            string base_filename = img_file.substr(img_file.find_last_of("/") + 1);
            
            string tmp = base_filename.substr(base_filename.find_last_of("_") + 1);
            string::size_type const p(tmp.find_last_of('.'));
            string time_stamp_str = tmp.substr(0, p); 
            
            double t_f = (stod(time_stamp_str)*1000000);
            frame.time = (long int)t_f%10000000000;
            frame.image_file = img_file; 

            dat_F.push_back(frame);           
            
        }  
        */  

      
        flag_init = 1;
        data_file_D.close();
        data_file_F.close();
              
    }
    //---------------------------------------------- 
    

    int idx;
    double current_t; 
    
    long int times[2] = {dat_F[0].time,dat_D[0].time};
    idx = distance(begin(times),min_element(begin(times), end(times)));
    current_t = times[idx]; 
    

    if ((dat_F.size() == 0) || (dat_D.size() == 0) )
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
                    //image = imread(Dataset_dir + Omni_dir + dat_F[0].image_file, IMREAD_GRAYSCALE  );
                    image = imread(Dataset_dir + Frontal_dir + dat_F[0].image_file, IMREAD_GRAYSCALE  );
                    
                   

                    dat.frame.image = image;
                    dat.frame.range = -1;                            
                    
                    dat_F.erase(dat_F.begin());
                    
                }
                else if(idx == 1)
                {   
                    dat.data_type = "odod";
                    dat.time = dat_D[0].time;
                    dat.odod = dat_D[0];
                    dat_D.erase(dat_D.begin());
                }                
                else
                {
                    
                }
                last_t = dat.time; // dataset time 
                last_time_data = std::chrono::high_resolution_clock::now();  // actual time 

                   
        }
           
    
    
    return dat;
}
