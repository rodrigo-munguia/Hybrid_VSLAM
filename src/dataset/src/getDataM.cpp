#include "getData.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include <armadillo>


/*---------------------------------------------------
Rodrigo Munguía 2020.

Get data from Bebop data base
-----------------------------------------------------
*/

using namespace std;
using namespace cv;


string frame_list1 = "frame_list.txt";
string Encoders_data = "Encoders.txt";

//-------------------------------------------------------------------------------------------------
// get data from Mobile Robot dataset
DATA getDataM(parameters &PAR)
{

 string Dataset_dir = PAR.Dataset_path;
    
    static int flag_init = 0;
    static vector<FRAME> dat_F;    
    static vector<ODOD> dat_E;
    DATA dat;
    
    static auto last_time_data = std::chrono::high_resolution_clock::now();
    static double last_t = 10000000000000000;
    
    if (PAR.restart == true)
    {      
       flag_init = 0;
       dat_F.clear();
       dat_E.clear();       
       last_time_data = std::chrono::high_resolution_clock::now();
       last_t = 10000000000000000;
       PAR.restart = false; 
    }


    
    // load data from the text files ----------
    // Execute only one time
    if (flag_init == 0)
    {   
        ifstream data_file_F;
        ifstream data_file_E;
          

        data_file_F.open(Dataset_dir + frame_list1); // frames file
        data_file_E.open(Dataset_dir + Encoders_data); // Encoders file
        

        string lineF;
        vector<string> v;
        while ( getline(data_file_F,lineF) )
        {  
            //stringstream ss(lineF);
            FRAME frame;
           // frame.time = stoi(lineF.substr(0,10));
            frame.time = stol(lineF.substr(0,10), nullptr, 10);
            frame.image_file = lineF;
            //int q = 10;
            if(frame.time > PAR.init_time && frame.time <  PAR.end_time)
                dat_F.push_back(frame);
        }
                
        string lineE;
        while ( getline(data_file_E,lineE) )
        {
            stringstream ss(lineE); 
            ODOD enc;
            vector<string> v_S;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_S.push_back( substr );
            }
            //range.time = stoi(v_R[0]);
            enc.time = stol(v_S[0], nullptr, 10);
            enc.TicksLeft = stod(v_S[1]);
            enc.TicksRight = stod(v_S[2]);         

            if(enc.time > PAR.init_time && enc.time <  PAR.end_time)
                dat_E.push_back(enc);
        }
        
        
        flag_init = 1;
        data_file_F.close();
        data_file_E.close();        
        
    }
    //----------------------------------------------   
    int idx;
    double current_t; 
    
    long int times[2] = {dat_F[0].time,dat_E[0].time};
    idx = distance(begin(times),min_element(begin(times), end(times)));
    current_t = times[idx]; 
   

    if ((dat_F.size() == 0) || (dat_E.size() == 0)  )
    {  
     //cout << "dataset: no more data" << endl;
     dat.data_type = "NULL";
     return dat;
    } 


            //int tmp = *min_element(times, end(times));
            // find the index of the minimun time
        
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
                    dat.frame.range = -1;
                    dat.frame.image = image;                                              
                    
                    dat_F.erase(dat_F.begin());
                    
                }
                else if(idx == 1)
                {   
                dat.data_type = "odod";
                dat.time = dat_E[0].time;
                dat.odod = dat_E[0];
                dat_E.erase(dat_E.begin());
                }                              
                else
                {
                    
                }
                last_t = dat.time; // dataset time 
                last_time_data = std::chrono::high_resolution_clock::now();  // actual time 

                   
        }
           
    
    
    return dat;
}



