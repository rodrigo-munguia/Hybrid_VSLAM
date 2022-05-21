// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "globalslam_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "../../common/Transforms/quat2R.hpp"
#include "gmap.hpp"
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "aux_functions.hpp"


using namespace std;

using std::placeholders::_1;

namespace globalslam
{


// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Gslam::Gslam(const rclcpp::NodeOptions & options)
: Node("gslam", options)
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  setParameters();

  // Set subscribers
  sub_Kf_ = this->create_subscription<interfaces::msg::Kf>("Kf_topic", 10, std::bind(&Gslam::Kf_callback, this, _1));
  sub_Kf_cl_ = this->create_subscription<interfaces::msg::Kf>("Kf_cl_topic", 10, std::bind(&Gslam::Kf_cl_callback, this, _1));  
  // Set services
  
  // Set clients
  client_local_slam_pos_update_ = this->create_client<interfaces::srv::LSposUpdate>("local_slam_pos_update");
  
  // Set publishers
  // Set publishers
  pub_gmap_data_ = create_publisher<interfaces::msg::Gmap>("gmap_data_topic",10);

  // set threads
  gmap_thread_loop_run = true;
  gmap_loop_ = thread(&Gslam::GMAP_LOOP,this);

  cloop_thread_loop_run = true;
  cloop_loop_ = thread(&Gslam::CLOOP_LOOP,this);

  gmap_updated = false;
  new_kf_cl = false;
  loop_closed_flag = false;
  
}
//--------------------------------------------------------------------------
Gslam::~Gslam()
  { 
    gmap_thread_loop_run = false;
    gmap_loop_.join();

  }

//------------------------------------------------------------------------
//  Main Global map loop (this function runs in a separate thread)
// Rodrigo M. 2022
void Gslam::GMAP_LOOP()
  {
   
    GMAP gmap(PAR); // create GMAP object  
    cout << "-> Global map thread running... " << endl; 
    
   
    while(gmap_thread_loop_run == true)  // thread main loop
    {

        bool new_kf = false;  
        //-- check if a loop has been closed ------------------
        mutex_get_gm.lock();
          if(loop_closed_flag == true)
          {
            loop_closed_flag = false;
            // Set the Global map with the copy updated by the closing loop process
            gmap.Set_GlobalMap(Gmap);
            // Reset kf waiting in buffer
            Kf_buffer.clear();
          }
        mutex_get_gm.unlock();
        //--------------------------------------------------------
        
        mutex_rx_kf.lock();           
          if(Kf_buffer.size() > 0)
          {
            for(int i = 0; i < Kf_buffer.size(); i++)
            {
              gmap.Add_Kf(Kf_buffer[i]);
            }
            Kf_buffer.clear(); // after adding the Kf, clear the buffer
            new_kf = true;
          }
        mutex_rx_kf.unlock();   

        if(new_kf == true)
        { 
          // if there are some new Keyframes update the global map
          gmap.Update();
          
         

          mutex_get_gm.lock();
            if(loop_closed_flag == false) // for synchronizing global map update after a close loop
            {
              gmap.Get_GlobalMap(Gmap); // Update the local copy of the Global Map
              //print_VG(Gmap );
              // publish global map data            
              Publish_gmap_data();
              gmap_updated = true;
            }           
          mutex_get_gm.unlock();
            
           
           if(loop_closed_flag == false) // for synchronizing global map update after a close loop
            {
              arma::vec::fixed<3> delta_pos = gmap.Get_delta_pos();
              mutex_send_pos.lock();
                Send_local_slam_pos_update(delta_pos,"bundle_adjustment");
              mutex_send_pos.unlock();
            }     

        }

      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep a short period of time to save proccesor use
    }  



  }
//-------------------------------------------------------------------
//  Close-loop  loop (this function runs in a separate thread)
// Rodrigo M. 2022
void Gslam::CLOOP_LOOP()
{
   
   CLOOP cloop(PAR); // create GMAP object  
   cout << "-> Close-loop thread running... " << endl; 



  while(cloop_thread_loop_run == true)  // thread main loop
    {   
      
      if(gmap_updated == true) // if global map has been updated, then get a local copy
      { 
        gmap_updated = false;        
        mutex_get_gm.lock();           
          cloop.Set_GlobalMap(Gmap);
        mutex_get_gm.unlock();        
      }
      if(new_kf_cl == true)
      {
          new_kf_cl = false;
          // carried out a close loop step
          mutex_rx_kf_cl.lock();
            KEYFRAME kf_cl = kf_cl_last;
          mutex_rx_kf_cl.unlock();
          
          cloop.Step(kf_cl);

          // If a loop has been closed -------------
          if(cloop.Get_close_loop_state() == true)
          {
            cloop.Set_close_loop_state(false);
            
            arma::vec::fixed<3> delta_pos = cloop.Get_delta_pos();
            mutex_send_pos.lock();            
              Send_local_slam_pos_update(delta_pos,"close_loop"); //  Update the pose of local slam process
            mutex_send_pos.unlock();  
            
            mutex_get_gm.lock(); 
              cloop.Get_GlobalMap(Gmap); // Update the local copy of the Global Map
              //Gmap.CL_flag = true;
              Publish_gmap_data();
              loop_closed_flag = true; // set the flag true
            mutex_get_gm.unlock();           

          }
          //----------------------------------------
      }     


       std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep a short period of time to save proccesor use 
    }    



}


//------------------------------------------------------------------
 void Gslam::Send_local_slam_pos_update(arma::vec::fixed<3> &Delta_kf_n,std::string type)
 {

   //cout << "delta_pos: " << Delta_kf_n << endl;

   auto request = std::make_shared<interfaces::srv::LSposUpdate::Request>();
   request->delta_x = Delta_kf_n[0];
   request->delta_y = Delta_kf_n[1];  
   request->delta_z = Delta_kf_n[2];
   request->type = type;             
   
   auto result = client_local_slam_pos_update_->async_send_request(request);

 }  
//----------------------------------------------------------------
// Publish GlobalMap data
void Gslam::Publish_gmap_data()
{
  auto message = interfaces::msg::Gmap();

  
  for(int i = 0; i < Gmap.KeyFDATA.size(); i++)
  {  
    
     arma::mat::fixed<3,3> Rn2r = Gmap.KeyFDATA[i].Rn2r;
     arma::mat::fixed<3,3> Rr2c = Gmap.KeyFDATA[i].Rr2c;
     arma::vec::fixed<3> r_N = Gmap.KeyFDATA[i].r_N;
     arma::vec::fixed<3> t_c2r = Gmap.KeyFDATA[i].t_c2r;
     arma::mat::fixed<3,3> Rn2c =  Gmap.KeyFDATA[i].Rn2c;
     arma::vec::fixed<3> t_c2n = Gmap.KeyFDATA[i].t_c2n;
     
     arma::vec::fixed<4> q_n2c; 
     Ra2b_TO_Quat_a2b_arma(Rn2c, q_n2c);

    geometry_msgs::msg::Transform t_n2c;
    t_n2c.translation.x = t_c2n(0);
    t_n2c.translation.y = t_c2n(1);
    t_n2c.translation.z = t_c2n(2);

    t_n2c.rotation.w =  q_n2c(0);
    t_n2c.rotation.x =  q_n2c(1);
    t_n2c.rotation.y =  q_n2c(2);
    t_n2c.rotation.z =  q_n2c(3);

    message.n2c.push_back(t_n2c);

  }

  for(int i = 0; i < Gmap.AnchorsDATA.size(); i++)
  { 
    geometry_msgs::msg::Point p; 
    p.x = Gmap.AnchorsDATA[i].AnchorState[0];
    p.y = Gmap.AnchorsDATA[i].AnchorState[1];
    p.z = Gmap.AnchorsDATA[i].AnchorState[2];
   message.gmap_anchors.push_back(p);
  } 


 pub_gmap_data_->publish(message); 
}
//--------------------------------------------------------------
// close loop "intermediate" KeyFrame callback
// Rodrigo M. 2022
void Gslam::Kf_cl_callback(const interfaces::msg::Kf & msg) 
  {
    KEYFRAME kf;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
    
    //cout << cv_ptr->image.cols << endl;

    kf.frame = cv_ptr->image;
    arma::vec::fixed<3> t_c2r;
    t_c2r(0) = msg.r2c.translation.x;
    t_c2r(1) = msg.r2c.translation.y;
    t_c2r(2) = msg.r2c.translation.z;
    kf.t_c2r = t_c2r;

    double quat[4];
    quat[0] = msg.r2c.rotation.w;
    quat[1] = msg.r2c.rotation.x;
    quat[2] = msg.r2c.rotation.y;
    quat[3] = msg.r2c.rotation.z;  
    double Rr2c_a[9];
    quat2R_col_major(quat,Rr2c_a);
    arma::mat Rr2c(Rr2c_a,3,3); // convert from arrat to armadillo

    kf.Rr2c = Rr2c;
    arma::vec::fixed<3> r_N;
    r_N(0) = msg.n2r.translation.x;
    r_N(1) = msg.n2r.translation.y;
    r_N(2) = msg.n2r.translation.z;
    kf.r_N = r_N;

    double quat2[4];
    quat2[0] = msg.n2r.rotation.w;
    quat2[1] = msg.n2r.rotation.x;
    quat2[2] = msg.n2r.rotation.y;
    quat2[3] = msg.n2r.rotation.z;  
    double Rn2r_a[9];
    quat2R_col_major(quat2,Rn2r_a);
    arma::mat Rn2r(Rn2r_a,3,3); // convert from arrat to armadillo

    kf.Rn2r = Rn2r;

    kf.Rn2c =  Rr2c*Rn2r;
    kf.t_c2n = r_N + Rn2r.t()*t_c2r;
    /*           
           cout << "New Keyframe received " << endl;
           cout << "r_N: " << kf.r_N << endl;
           cout << "Rn2r: " << Rn2r << endl;
           cout << "t_c2r: " << kf.t_c2r << endl;
           cout << "Rr2c: " << Rr2c << endl;
    */      

    mutex_rx_kf_cl.lock(); 
      kf_cl_last = kf;
       new_kf_cl = true;
    mutex_rx_kf_cl.unlock();     

  }  

//---------------------------------------------------------------
// KeyFrame callback
// Rodrigo M. 2022
void Gslam::Kf_callback(const interfaces::msg::Kf & msg) const
  {
    KEYFRAME kf;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
    
    //cout << cv_ptr->image.cols << endl;

    kf.frame = cv_ptr->image;
    arma::vec::fixed<3> t_c2r;
    t_c2r(0) = msg.r2c.translation.x;
    t_c2r(1) = msg.r2c.translation.y;
    t_c2r(2) = msg.r2c.translation.z;
    kf.t_c2r = t_c2r;

    double quat[4];
    quat[0] = msg.r2c.rotation.w;
    quat[1] = msg.r2c.rotation.x;
    quat[2] = msg.r2c.rotation.y;
    quat[3] = msg.r2c.rotation.z;  
    double Rr2c_a[9];
    quat2R_col_major(quat,Rr2c_a);
    arma::mat Rr2c(Rr2c_a,3,3); // convert from arrat to armadillo

    kf.Rr2c = Rr2c;
    arma::vec::fixed<3> r_N;
    r_N(0) = msg.n2r.translation.x;
    r_N(1) = msg.n2r.translation.y;
    r_N(2) = msg.n2r.translation.z;
    kf.r_N = r_N;

    double quat2[4];
    quat2[0] = msg.n2r.rotation.w;
    quat2[1] = msg.n2r.rotation.x;
    quat2[2] = msg.n2r.rotation.y;
    quat2[3] = msg.n2r.rotation.z;  
    double Rn2r_a[9];
    quat2R_col_major(quat2,Rn2r_a);
    arma::mat Rn2r(Rn2r_a,3,3); // convert from arrat to armadillo

    kf.Rn2r = Rn2r;

    kf.Rn2c =  Rr2c*Rn2r;
    kf.t_c2n = r_N + Rn2r.t()*t_c2r;


          /* 
           cout << "New Keyframe received " << endl;
           cout << "r_N: " << kf.r_N << endl;
           cout << "Rn2r: " << Rn2r << endl;
           cout << "t_c2r: " << kf.t_c2r << endl;
           cout << "Rr2c: " << Rr2c << endl;
          */

    mutex_rx_kf.lock(); 
      Kf_buffer.push_back(kf);
    mutex_rx_kf.unlock();     



  }  






//-------------------------------------------------------------
}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(globalslam::Gslam)