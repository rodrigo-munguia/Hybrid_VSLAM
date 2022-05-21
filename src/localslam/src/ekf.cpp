#include "ekf.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "../../common/Transforms/AngleWrap.hpp"


typedef std::vector<double> vec_t;


void EKF::system_init(DATA &dat)
{

    if(PAR.Data_origin == "dataset")
    {
       
       if (dat.data_type == "attitude")
       {
          //init_quat_system_state(dat);
          init_euler_system_statet(dat);
          initialized = true;
       }       


    }

}
//----------------------------------------------
void EKF::init_euler_system_statet(DATA &dat)
{
  
  x.resize(PAR.Robot_state_size);
  P.resize(PAR.Robot_state_size,PAR.Robot_state_size);
  x.zeros();
  P.zeros();

  yaw_at_home = dat.att.yaw;
  AngleWrap(yaw_at_home);

  double axis_x = PAR.Robot_init_roll ;
  double axis_y = PAR.Robot_init_pitch ;
  double axis_z = PAR.Robot_init_yaw;
  //double axis_z = yaw_at_home;
  double x_init = PAR.Robot_init_x;
  double y_init = PAR.Robot_init_y;
  double z_init = PAR.Robot_init_z;

  x(0) = 0; 
  x(1) = axis_x;
  x(2) = axis_y;
  x(3) = axis_z;
  x(4) = 0;
  x(5) = 0;
  x(6) = 0;
  x(7) = x_init;
  x(8) = y_init;
  x(9) = z_init;
  x(10) = 0;
  x(11) = 0;
  x(12) = 0;

  //cout << "x_0 norm " << arma::norm(x) << endl;
  cout << "-> robot state vector initialized " << endl;

  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*numeric_limits<double>::epsilon();
  double eps = numeric_limits<double>::epsilon()  ;
  arma::vec::fixed<13> p_ini = {0, .01, .01, .01, .01, .01, .01, eps, eps, eps, .0001, .0001, .0001};

  //-------------------------------------------------------------------------------  
  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
         
  t_c2r(0) = PAR.Mono_cam_2_robot_pos_x;
  t_c2r(1) = PAR.Mono_cam_2_robot_pos_y;
  t_c2r(2) = PAR.Mono_cam_2_robot_pos_z;        

  double cam_axis_x = PAR.Mono_cam_2_robot_axis_x;
  double cam_axis_y = PAR.Mono_cam_2_robot_axis_y;
  double cam_axis_z = PAR.Mono_cam_2_robot_axis_z;
  double Ra2b_c[9];
  Euler_to_Ra2b_colum_major(cam_axis_x, cam_axis_y, cam_axis_z, Ra2b_c);
  arma::mat Rr2c_t(Ra2b_c,3,3); // camera to robot rotation matrix 
  Rr2c = Rr2c_t;
  //-------------------------------------------------------------------------------- 

  FeatsDATA.clear();
  AnchorsDATA.clear();
  NewKF_available = false;
  NewRobotState_available = false;
  New_KF_cl = false;

 
 
}
//----------------------------------------------------------
void EKF::init_quat_system_state(DATA &dat)
{ 
  x.resize(PAR.Robot_state_size);
  P.resize(PAR.Robot_state_size,PAR.Robot_state_size);
  x.zeros();
  P.zeros();

  yaw_at_home = dat.att.yaw;
  AngleWrap(yaw_at_home);

  double axis_x = PAR.Robot_init_roll ;
  double axis_y = PAR.Robot_init_pitch ;
  double axis_z = PAR.Robot_init_yaw;
  //double axis_z = yaw_at_home;
  double x_init = PAR.Robot_init_x;
  double y_init = PAR.Robot_init_y;
  double z_init = PAR.Robot_init_z;
  double Ra2b[9];

  Euler_to_Ra2b_colum_major(axis_x, axis_y, axis_z, Ra2b);    

  double q_int[4];
  Ra2b_TO_Quat_a2b(Ra2b,q_int);

  x(0) = q_int[0];
  x(1) = q_int[1];
  x(2) = q_int[2];
  x(3) = q_int[3];
  x(4) = numeric_limits<double>::epsilon();
  x(5) = numeric_limits<double>::epsilon();
  x(6) = numeric_limits<double>::epsilon();
  x(7) = x_init;
  x(8) = y_init;
  x(9) = z_init;
  x(10) = numeric_limits<double>::epsilon();
  x(11) = numeric_limits<double>::epsilon();
  x(12) = numeric_limits<double>::epsilon();

  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*numeric_limits<double>::epsilon();
  double eps = numeric_limits<double>::epsilon()  ;
  arma::vec::fixed<13> p_ini = {eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps, eps};
  
  //P = eye(PAR.Robot_state_size,PAR.Robot_state_size)*p_ini;
  P = arma::diagmat(p_ini);
  
  //-------------------------------------------------------------------------------  
  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;
         
  t_c2r(0) = PAR.Mono_cam_2_robot_pos_x;
  t_c2r(1) = PAR.Mono_cam_2_robot_pos_y;
  t_c2r(2) = PAR.Mono_cam_2_robot_pos_z;        

  double cam_axis_x = PAR.Mono_cam_2_robot_axis_x;
  double cam_axis_y = PAR.Mono_cam_2_robot_axis_y;
  double cam_axis_z = PAR.Mono_cam_2_robot_axis_z;
  double Ra2b_c[9];
  Euler_to_Ra2b_colum_major(cam_axis_x, cam_axis_y, cam_axis_z, Ra2b_c);
  arma::mat Rr2c_t(Ra2b_c,3,3); // camera to robot rotation matrix 
  Rr2c = Rr2c_t;


  //-------------------------------------------------------------------------------- 
  FeatsDATA.clear();
  AnchorsDATA.clear();
  NewKF_available = false;
  NewRobotState_available = false;
  New_KF_cl = false;

  cout << "lslam state initialized" << endl;
  cout << x << endl;

  

}
//----------------------------------------------
void EKF::ekf_step(DATA &dat)
{   
    
    static auto ct_i = std::chrono::high_resolution_clock::now(); // for computation time 
    
    static auto dt_i = std::chrono::high_resolution_clock::now(); // for compute delta_t and 
    static auto last_time = std::chrono::high_resolution_clock::now();
    static auto last_time_data = std::chrono::high_resolution_clock::now();
    
    static double last_timeDS;
    static bool init_t = false;
    
    static double total_comp_time = 0;
    static double total_exec_time = 0;

    static int n_o = 0;

        //----- ekf prediction --------------------------------------------------------------------
            
            dt_i = std::chrono::high_resolution_clock::now();            
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(dt_i - last_time);            
            double  delta_t = elapsed.count() * 1e-9;
            if(delta_t > PAR.Mono_cam_frame_period)delta_t = PAR.Mono_cam_frame_period;
            bool prediction_done = false;
            
            if(PAR.Data_origin == "dataset")                
                { 
                  delta_t = delta_t*PAR.x_vel_run_time;
                } 
                
            if( delta_t > 1/(double)PAR.Max_freq_output || dat.data_type != "")
            { 
                
                bool receiving_data;
                if(dat.data_type != "")
                {
                  last_time_data = std::chrono::high_resolution_clock::now();  
                }
                
                auto ct_d = std::chrono::high_resolution_clock::now();
                auto elapsed_last_data = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_d - last_time_data);
                double et_since_last_data = elapsed_last_data .count() * 1e-9; 
                if(et_since_last_data > .1)
                { 
                    // if no data has been received by a period o time, set flag
                    receiving_data = false;
                }
                else
                {
                    receiving_data = true;
                }
                
              ct_i = std::chrono::high_resolution_clock::now();  
             // cout << delta_t << endl;
              
              if(receiving_data)prediction(delta_t); // if data is been received, perform EKF prediction

              total_exec_time =  total_exec_time + delta_t;
              prediction_done = true;
              last_time = dt_i;
            } 
            
            //-------------- end EKF prediction ------------------------------------------------------
           
            if (dat.data_type != "") // for any kind of measurement
            {                 
               // n_o++;            
                /*
                if(PAR.Data_origin == "dataset")
                {
                  // compute delta_t from dataset times
                  if (init_t == false)
                  {
                    delta_t = PAR.Mono_cam_frame_period;
                    last_timeDS = dat.time;
                    init_t = true;
                  }
                  else
                  {
                    delta_t = (dat.time - last_timeDS)/1000000;
                    last_timeDS = dat.time;                    
                  }                 

                }
                */                  
                /*
                ct_i = std::chrono::high_resolution_clock::now();    

                auto t_c = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_c - last_time);
                double  delta_t = elapsed.count() * 1e-9;
                if(delta_t > PAR.Mono_cam_frame_period)delta_t = PAR.Mono_cam_frame_period; // set the maximun delta_t as the camera frame rate period 
                last_time = t_c;
                // for data from dataset, compensate for the data frequency of the dataset component
                
                if(PAR.Data_origin == "dataset")                
                { 
                  delta_t = delta_t*PAR.x_vel_run_time;
                }  

                prediction(delta_t); // EKF prediction
                
                total_exec_time =  total_exec_time + delta_t;
                //cout << delta_t <<  endl;
                */
            } 
                 

            if(dat.data_type == "frame")
            {
              //cout << "Frame rows: " << dat.frame.image.rows << endl;
               if(PAR.Visual_update)Visual_update_e(&dat.frame);

               auto ct_f = std::chrono::high_resolution_clock::now();
               auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
               double comp_time_per_frame = elapsed.count() * 1e-9;
               
              // cout << "computation time per frame(s): " << comp_time_per_frame << endl;
              // cout << "total exec time(s): " << total_exec_time << " total comp time(s): " << total_comp_time << endl;
              // cout << n_o << endl;
            }

            if(dat.data_type == "alt")
            {
              //cout << "Altitude:" << dat.alt.altitude << endl;
              if(PAR.Altitude_update)Altitude_update(dat.alt);              

            }
            if(dat.data_type == "speed")
            {
              //cout << "Speed x: " << dat.spd.speedX << endl;
              //cout << "position before update : " << x(7) << " " << x(8) << " " << x(9) << endl;
              if(PAR.Speed_update)Speed_Update_nav_euler(dat.spd);
              //cout << "position after update : " << x(7) << " " << x(8) << " " << x(9) << endl; 

            }
            if(dat.data_type == "attitude")
            {       
              
              //cout << "roll: " << dat.att.roll*(180/3.1416) <<  " pitch: " << dat.att.pitch*(180/3.1416)  << " yaw: " << dat.att.yaw*(180/3.1416) <<endl;
              //cout << "sum_dt " << sum_dt << endl; 
              //cout << "b: " << x(0) << " " << x(1)*(180/3.1416) << " " << x(2)*(180/3.1416) << " "<<  x(3)*(180/3.1416) << endl;
              if(PAR.Attitude_update)Attitude_euler_Update(dat.att);
              //cout << "a: " << x(0) << " " << x(1)*(180/3.1416) << " " << x(2)*(180/3.1416) << " "<<  x(3)*(180/3.1416) << endl;              

            }
            if(dat.data_type == "range")
            {
            
            //cout << "Range: " << dat.range.range << endl;

            }
            if(dat.data_type == "gps")
            {
            
            //cout << "Gps sats: " << dat.gps.sat << endl;

            }
            if (dat.data_type != "" || prediction_done == true ) // for any kind of measurement
            {   
               auto ct_t = std::chrono::high_resolution_clock::now();
               auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_t - ct_i);
               double comp_time_per_step = elapsed.count() * 1e-9;            
               total_comp_time = total_comp_time + comp_time_per_step;
               n_o++;
              
               NewRobotState_available = true;
            }


}

//----------------------------------------------------
void EKF::Update_pos_with_delta(arma::vec::fixed<3> delta_pos, string type)
{ 
  
  double innov_xy = sqrt(pow(delta_pos[0],2) + pow(delta_pos[1],2)); 

  if (type == "bundle_adjustment")
  { 
     //--------------------------------------------------------------
    //cout << "EKF-SLAM -> BA update" << endl;
    // update robot position
    if(PAR.GS_xy_update == true && innov_xy < PAR.GS_xy_update_max_delta )
    {
      x[7] = x[7] + delta_pos[0];
      x[8] = x[8] + delta_pos[1];
    }
    if(PAR.GS_z_update == true && delta_pos[2] < PAR.GS_z_update_max_delta)
    {  
      x[9] = x[9] + delta_pos[2];
    }  
    // update Features position
    for(int i = 0; i< FeatsDATA.size();i++ )
    {
      int idx_i = FeatsDATA[i].idx_i_state;
      if(PAR.GS_xy_update == true && innov_xy < PAR.GS_xy_update_max_delta )
      {
        x[idx_i] = x[idx_i] + delta_pos[0];
        x[idx_i+1] = x[idx_i+1] + delta_pos[1];
      }
      if(PAR.GS_z_update == true && delta_pos[2] < PAR.GS_z_update_max_delta)
      {   
        x[idx_i+2] = x[idx_i+2] + delta_pos[2];
      }  
    }
    // Updates Anchors position
    for(int i = 0; i< AnchorsDATA.size();i++)
    { 
      if(PAR.GS_xy_update == true && innov_xy < PAR.GS_xy_update_max_delta )
      {
        AnchorsDATA[i].AnchorState[0]  =  AnchorsDATA[i].AnchorState[0] + delta_pos[0];
        AnchorsDATA[i].AnchorState[1]  =  AnchorsDATA[i].AnchorState[1] + delta_pos[1];
      }
      if(PAR.GS_z_update == true && delta_pos[2] < PAR.GS_z_update_max_delta)
      { 
        AnchorsDATA[i].AnchorState[2]  =  AnchorsDATA[i].AnchorState[2] + delta_pos[2];
      }  
    }
    //---------------------------------------------------------------------     
  }
  else if(type == "close_loop")
  {
    //cout << "EKF-SLAM -> CL update" << endl;
    // update robot position
    //cout << "ekf deltapos : " << delta_pos << endl; 
   // cout << "pos b: " << x[7] << " " << x[8] << " " << x[9] << endl;
    if(PAR.CL_xy_update == true && innov_xy < PAR.CL_xy_update_max_delta )
    {
      x[7] = x[7] + delta_pos[0];
      x[8] = x[8] + delta_pos[1];
    }
    if(PAR.CL_z_update == true && delta_pos[2] < PAR.CL_z_update_max_delta)
    {  
      x[9] = x[9] + delta_pos[2];
    }
    //cout << "pos a: " << x[7] << " " << x[8] << " " << x[9] << endl;  
    // update Features position
    for(int i = 0; i< FeatsDATA.size();i++ )
    {
      int idx_i = FeatsDATA[i].idx_i_state;
      if(PAR.CL_xy_update == true && innov_xy < PAR.CL_xy_update_max_delta )
      {
        x[idx_i] = x[idx_i] + delta_pos[0];
        x[idx_i+1] = x[idx_i+1] + delta_pos[1];
      }
      if(PAR.CL_z_update == true && delta_pos[2] < PAR.CL_z_update_max_delta)
      {   
        x[idx_i+2] = x[idx_i+2] + delta_pos[2];
      }  
    }
    // Updates Anchors position
    for(int i = 0; i< AnchorsDATA.size();i++)
    { 
      if(PAR.CL_xy_update == true && innov_xy < PAR.CL_xy_update_max_delta )
      {
        AnchorsDATA[i].AnchorState[0]  =  AnchorsDATA[i].AnchorState[0] + delta_pos[0];
        AnchorsDATA[i].AnchorState[1]  =  AnchorsDATA[i].AnchorState[1] + delta_pos[1];
      }
      if(PAR.CL_z_update == true && delta_pos[2] < PAR.CL_z_update_max_delta)
      { 
        AnchorsDATA[i].AnchorState[2]  =  AnchorsDATA[i].AnchorState[2] + delta_pos[2];
      }  
    }
    //---------------------------------------------------------------------    
  }


  //cout << "delta_pos update request: " << delta_pos << endl;
  
  
  

}

//--------------------------------------
bool EKF::get_ekf_initialized_state()
{
    return initialized;
}
void EKF::set_ekf_initialized_statet(bool state)
{
    initialized = state;
} 
//-----------------------------------------
// Rodrigo M. 2022
bool EKF::get_RobotState(arma::vec::fixed<13> &x_r)
{
  if(NewRobotState_available == true)
  {
    x_r = x.subvec(0,12);
    NewRobotState_available = false;
    return true;
  }
  else
  {
    return false;
  }

}
//-----------------------------------------
// Rodrigo M. 2022
LOCALSLAM_DATA EKF::get_lslam_data()
{
  LOCALSLAM_DATA slam_state;
  
  // get robot state
  
  slam_state.robot_state = arma::conv_to<vec_t>::from(x.subvec(0,PAR.Robot_state_size-1)); // convert from armadillo vector to c++ vector 
  
  slam_state.Rr2c = Rr2c;
  slam_state.t_c2r = t_c2r;

  double phi = x(1);
  double theta = x(2);
  double psi = x(3);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
  arma::mat Rn2r(Ra2b,3,3);
  
  slam_state.Rn2r = Rn2r;
  slam_state.r_N = x.subvec(7,9); 

  // get map points
  slam_state.map_state.clear();
  slam_state.matched_img_feats.clear();
  slam_state.unmatched_img_feats.clear();
  slam_state.unmatched_img_anchors.clear();
  slam_state.matched_img_anchors.clear();  
  
  for(int i = 0; i < FeatsDATA.size(); i++)
  {
    POINT3d p;
    p.x =  x(FeatsDATA[i].idx_i_state);
    p.y =  x(FeatsDATA[i].idx_i_state + 1);
    p.z =  x(FeatsDATA[i].idx_i_state + 2);
    slam_state.map_state.push_back(p);
    
    if (FeatsDATA[i].predicted == true)
    {
      if(FeatsDATA[i].matched == true )
      {
        POINT3d pm;
        pm.x = FeatsDATA[i].Keypoint.pt.x;
        pm.y = FeatsDATA[i].Keypoint.pt.y;
        slam_state.matched_img_feats.push_back(pm);
      }
      else
      { 
        POINT3d pu;
        pu.x = FeatsDATA[i].PredictedPoint.x;
        pu.y = FeatsDATA[i].PredictedPoint.y;
        slam_state.unmatched_img_feats.push_back(pu);
      }
    }
  }
//-----------------------------------------------
 for(int i = 0; i < AnchorsDATA.size(); i++)
  {   
      POINT3d p;
      p.x =  AnchorsDATA[i].AnchorState[0];
      p.y =  AnchorsDATA[i].AnchorState[1];
      p.z =  AnchorsDATA[i].AnchorState[2];      
      slam_state.map_anchors.push_back(p);

      if (AnchorsDATA[i].predicted == true)
      {
        if(AnchorsDATA[i].matched == true )
        {
          POINT3d pm;
          pm.x = AnchorsDATA[i].Keypoint.pt.x;
          pm.y = AnchorsDATA[i].Keypoint.pt.y;
          slam_state.matched_img_anchors.push_back(pm);
        }
        else
        { 
          POINT3d pu;
          pu.x = AnchorsDATA[i].PredictedPoint.x;
          pu.y = AnchorsDATA[i].PredictedPoint.y;
          slam_state.unmatched_img_anchors.push_back(pu);
        }
      }
  }       

 
  return slam_state;
}

//----------------------------------------------------------------
bool EKF::get_KeyFrame(KEYFRAME &KF)
{
  if(NewKF_available == true)
  {
    KF = KF_selected;
    NewKF_available = false;
    return true;
  }
  else
  {
    return false;
  }

}
//--------------------------------------------------------------
bool EKF::get_KeyFrameCL(KEYFRAME &KF)
{
  if(New_KF_cl == true)
  {
    KF = KF_cl;
    New_KF_cl = false;
    return true;
  }
  else
  {
    return false;
  }

}
