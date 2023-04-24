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

#include "localslam_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cinttypes>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/lslam.hpp"
#include "interfaces/msg/kf.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "geometry_msgs/msg/point.h"
#include "ekf.hpp"
//#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "../../common/Transforms/Ra2b_TO_Quat_a2b.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include <cv_bridge/cv_bridge.h>

//#include <Matrix3x3.h>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

namespace localslam
{


// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
EKFslam::EKFslam(const rclcpp::NodeOptions & options)
: Node("lslam", options)
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  setParameters();

  // Set subscribers
  sub_Alt_ = this->create_subscription<interfaces::msg::Alt>("Altitude_topic", 10, std::bind(&EKFslam::Alt_callback, this, _1)); 
  sub_Att_ = this->create_subscription<interfaces::msg::Att>("Attitude_topic", 10, std::bind(&EKFslam::Att_callback, this, _1));
  sub_Gps_ = this->create_subscription<interfaces::msg::Gps>("Gps_topic", 10, std::bind(&EKFslam::Gps_callback, this, _1)); 
  sub_Range_ = this->create_subscription<interfaces::msg::Range>("Range_topic", 10, std::bind(&EKFslam::Range_callback, this, _1));
  sub_Frame_ = this->create_subscription<interfaces::msg::Frame>("Frame_topic", 10, std::bind(&EKFslam::Frame_callback, this, _1));        
  sub_Spd_ = this->create_subscription<interfaces::msg::Spd>("Speed_topic", 10, std::bind(&EKFslam::Speed_callback, this, _1));        
  sub_OdoD_ = this->create_subscription<interfaces::msg::Ododiff>("OdometryD_topic",10, std::bind(&EKFslam::OdoDiff_callback, this, _1)); 
  sub_OdoV_ = this->create_subscription<interfaces::msg::Odovw>("OdometryV_topic",10, std::bind(&EKFslam::OdoVW_callback, this, _1)); 
  // Set services
  srv_ekf_run_ = this->create_service<interfaces::srv::SimpleServ>("ekf_run_service",std::bind(&EKFslam::Handle_ekf_run_service, this,_1,_2));
  srv_pos_update_ = this->create_service<interfaces::srv::LSposUpdate>("local_slam_pos_update",std::bind(&EKFslam::Handle_pos_update_service, this,_1,_2)  );

  ekf_thread_loop_run = true; // main ekf  loop flag 
  ekf_loop_ = thread(&EKFslam::EKF_LOOP,this);
  
  // Set publishers
  pub_lslam_data_ = create_publisher<interfaces::msg::Lslam>("lslam_data_topic",10);
  pub_kf_ = create_publisher<interfaces::msg::Kf>("Kf_topic",10);pub_kf_ = create_publisher<interfaces::msg::Kf>("Kf_topic",10);
  pub_kf_cl_ = create_publisher<interfaces::msg::Kf>("Kf_cl_topic",10);
  pub_robot_state_ =  create_publisher<interfaces::msg::Robotstate>("Robotstate_topic",10);

    

  // Use a timer to schedule periodic message publishing.
  new_slam_state_flag = false;
  //timer_ = create_wall_timer(33ms, std::bind(&EKFslam::on_timer_pub_lslam_data, this));

  re_init_sys = false;
  ekf_steps_loop_run = false;

 // version info.
  std::cout << cv::getBuildInformation() << std::endl;
  arma::arma_version ver;
  std::cout << "ARMA version: "<< ver.as_string() << std::endl;
  
}
//--------------------------------------------------------------------------
EKFslam::~EKFslam()
{ 
  ekf_thread_loop_run = false; // for leaving the main ekf loop
  ekf_loop_.join();
}
//-----------------------------------------------------------------------------


//------------------------------------------------------------------------
//  Main EKF slam loop (this function runs in a separate thread)
void EKFslam::EKF_LOOP()
{
  
  EKF ekf(PAR); // create EKF object  
  cout << "-> EKF-SLAM thread running... " << endl;   

  DATA dat;
  bool ekf_s_l_r = false;  
  
  while(ekf_thread_loop_run == true)  // thread main loop
  {
    
    mutex_ekf_run.lock();
      ekf_s_l_r = ekf_steps_loop_run;
    mutex_ekf_run.unlock();      


    while(ekf_s_l_r == true)  // if ekf_steps_loop_run flag is true
    { 
        mutex_ekf_run.lock();
        // if a signal to stop/start the loop of steps is received
          ekf_s_l_r = ekf_steps_loop_run; // this flag can be changed by service "srv_ekf_run_"
        mutex_ekf_run.unlock(); 
       
        mutex_init_system.lock();
          if (re_init_sys == true) // this flag can be changed by service "srv_ekf_run_"
          { 
            // tell the ekf object that the system must be initialized again
            ekf.set_ekf_initialized_statet(false);         
            re_init_sys = false;
          }      
        mutex_init_system.unlock();  

      if (ekf.get_ekf_initialized_state() == false)
        {   
            // Initialize system state
            dat = getData();
            ekf.system_init(dat);           
        }
      else
        {          
            //--------------------            
            dat = getData();  // get data from dataset/actual robot           
            
            ekf.ekf_step(dat); // perform a filter step
            
            //------- after ekf step publish stuff!!! --------------------------------------
                // check if the robot state has been updated, if so, then publish
                arma::vec::fixed<13> x_r; 
                if (ekf.get_RobotState(x_r) == true)
                {
                  pub_robot_state(x_r);
                }        

                if (dat.data_type == "frame")
                {
                  // for each frame check if there is a new Key Frame available
                  KEYFRAME KF;
                  if(ekf.get_KeyFrame(KF) == true)
                  {
                    // publish keyframe for global mapping process
                    pub_kf(KF);
                  } 
                  KEYFRAME KFCL;
                  if(ekf.get_KeyFrameCL(KFCL) == true)
                  {
                    // publish "intermediate" keyframe for closing-loop process
                    pub_kf_cl(KFCL);
                  }
                    // publish local slam data             
                    LOCALSLAM_DATA lslam_data;
                    lslam_data = ekf.get_lslam_data();
                    pub_lslam_Data(lslam_data);
                }
             //-------------end publish data ----------------------------------------
             
             // -- check if a position update is available from the global map component
             mutex_pos_update.lock();
              if(pos_update.pos_update_available == true)
              {
                pos_update.pos_update_available = false;
                ekf.Update_pos_with_delta(pos_update.delta_pos_update,pos_update.type);
              }
             mutex_pos_update.unlock();


        
        } // if ekf is initilized

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // sleep a short period of time to save proccesor use

    } // while(ekf.run == true)    

  }
 

}
//--------------------------------------------------------------------------------------------------------------
// Publish robot state data to topic
// Rodrigo M. 2022
void EKFslam::pub_robot_state(arma::vec::fixed<13> &x_r)
{
  auto message = interfaces::msg::Robotstate();

  message.robot_state = arma::conv_to<std::vector<double>>::from(x_r);

  double phi = x_r(1);
  double theta = x_r(2);
  double psi = x_r(3);
  double Ra2b[9];
  Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
  arma::mat Rn2r_a(Ra2b,3,3);
  arma::mat::fixed<3,3> Rn2r = Rn2r_a;
  arma::vec::fixed<4> q_n2r; 
  Ra2b_TO_Quat_a2b_arma(Rn2r, q_n2r);

  geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = x_r(7);
    t_n2r.translation.y = x_r(8);
    t_n2r.translation.z = x_r(9);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

  message.n2r = t_n2r;
  
  geometry_msgs::msg::Twist vels;
  // linear velocity of the robot expressed in the body frame
  vels.linear.x = x_r(10);
  vels.linear.y = x_r(11);
  vels.linear.z = x_r(12);
  // angular velocity of the robot expressed in the body frame
  vels.angular.x = x_r(4);
  vels.angular.y = x_r(5);
  vels.angular.z = x_r(6);

  message.vels = vels;

  pub_robot_state_->publish(message);  

}

//--------------------------------------------------------------------------------------------------------------
// Publish LocalSLAM data to topic
// Rodrigo M. 2022
void EKFslam::pub_lslam_Data(LOCALSLAM_DATA &lslam_data)
{
  auto message = interfaces::msg::Lslam();      
           
      int size_map = lslam_data.map_state.size();
     
      message.map_state.clear();      
      message.robot_state = lslam_data.robot_state;

      arma::vec::fixed<4> q_r2c; 
     Ra2b_TO_Quat_a2b_arma(lslam_data.Rr2c, q_r2c); 
  
     geometry_msgs::msg::Transform t_r2c;
    t_r2c.translation.x = lslam_data.t_c2r(0);
    t_r2c.translation.y = lslam_data.t_c2r(1);
    t_r2c.translation.z = lslam_data.t_c2r(2);

    t_r2c.rotation.w =  q_r2c(0);
    t_r2c.rotation.x =  q_r2c(1);
    t_r2c.rotation.y =  q_r2c(2);
    t_r2c.rotation.z =  q_r2c(3);

    arma::vec::fixed<4> q_n2r; 
    Ra2b_TO_Quat_a2b_arma(lslam_data.Rn2r, q_n2r);

    geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = lslam_data.r_N(0);
    t_n2r.translation.y = lslam_data.r_N(1);
    t_n2r.translation.z = lslam_data.r_N(2);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

    message.r2c = t_r2c;
    message.n2r = t_n2r;         
         
         for(int i = 0; i < size_map ; i++)
         { 
            geometry_msgs::msg::Point p;        
            p.x = lslam_data.map_state[i].x;
            p.y = lslam_data.map_state[i].y;
            p.z = lslam_data.map_state[i].z;
            message.map_state.push_back(p);
         }
         
         for(int i = 0; i < lslam_data.matched_img_feats.size() ; i++)
         {
            geometry_msgs::msg::Point pm;
            pm.x = lslam_data.matched_img_feats[i].x;
            pm.y = lslam_data.matched_img_feats[i].y;
            message.matched_img_feats.push_back(pm);
         }
           
         for(int i = 0; i < lslam_data.unmatched_img_feats.size() ; i++)
         {
            geometry_msgs::msg::Point pu;
            pu.x = lslam_data.unmatched_img_feats[i].x;
            pu.y = lslam_data.unmatched_img_feats[i].y;
            message.unmatched_img_feats.push_back(pu);
         }

         for(int i = 0; i < lslam_data.map_anchors.size() ; i++)
         { 
            geometry_msgs::msg::Point p;        
            p.x = lslam_data.map_anchors[i].x;
            p.y = lslam_data.map_anchors[i].y;
            p.z = lslam_data.map_anchors[i].z;
            message.map_anchors.push_back(p);
         }

         for(int i = 0; i < lslam_data.matched_img_anchors.size() ; i++)
         {
            geometry_msgs::msg::Point pm;
            pm.x = lslam_data.matched_img_anchors[i].x;
            pm.y = lslam_data.matched_img_anchors[i].y;
            message.matched_img_anchors.push_back(pm);
         }
           
         for(int i = 0; i < lslam_data.unmatched_img_anchors.size() ; i++)
         {
            geometry_msgs::msg::Point pu;
            pu.x = lslam_data.unmatched_img_anchors[i].x;
            pu.y = lslam_data.unmatched_img_anchors[i].y;
            message.unmatched_img_anchors.push_back(pu);
         }       
     
  pub_lslam_data_->publish(message);    
    //cout << "xx" << endl;


}
//----------------------------------------------------------------------------------------------------------------
// Publish "intermediate"KeyFrames to topic
// Rodrigo M. 2022
void EKFslam::pub_kf_cl(KEYFRAME &KF)
{ 
  
  arma::vec::fixed<4> q_r2c; 
  Ra2b_TO_Quat_a2b_arma(KF.Rr2c, q_r2c); 
  
    geometry_msgs::msg::Transform t_r2c;
    t_r2c.translation.x = KF.t_c2r(0);
    t_r2c.translation.y = KF.t_c2r(1);
    t_r2c.translation.z = KF.t_c2r(2);

    t_r2c.rotation.w =  q_r2c(0);
    t_r2c.rotation.x =  q_r2c(1);
    t_r2c.rotation.y =  q_r2c(2);
    t_r2c.rotation.z =  q_r2c(3);

  arma::vec::fixed<4> q_n2r; 
  Ra2b_TO_Quat_a2b_arma(KF.Rn2r, q_n2r);

    geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = KF.r_N(0);
    t_n2r.translation.y = KF.r_N(1);
    t_n2r.translation.z = KF.r_N(2);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

  auto message = interfaces::msg::Kf();
    message.r2c = t_r2c;
    message.n2r = t_n2r;
  
  sensor_msgs::msg::Image::SharedPtr img_msg; 
  img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, KF.frame).toImageMsg();        
  message.img = *img_msg;

  pub_kf_cl_->publish(message);
  //cout << KF.r_N << endl;

}

//--------------------------------------------------------------------------------------------------------------
// Publish KeyFrames to topic
// Rodrigo M. 2022
void EKFslam::pub_kf(KEYFRAME &KF)
{ 
  
  arma::vec::fixed<4> q_r2c; 
  Ra2b_TO_Quat_a2b_arma(KF.Rr2c, q_r2c); 
  
    geometry_msgs::msg::Transform t_r2c;
    t_r2c.translation.x = KF.t_c2r(0);
    t_r2c.translation.y = KF.t_c2r(1);
    t_r2c.translation.z = KF.t_c2r(2);

    t_r2c.rotation.w =  q_r2c(0);
    t_r2c.rotation.x =  q_r2c(1);
    t_r2c.rotation.y =  q_r2c(2);
    t_r2c.rotation.z =  q_r2c(3);

  arma::vec::fixed<4> q_n2r; 
  Ra2b_TO_Quat_a2b_arma(KF.Rn2r, q_n2r);

    geometry_msgs::msg::Transform t_n2r;
    t_n2r.translation.x = KF.r_N(0);
    t_n2r.translation.y = KF.r_N(1);
    t_n2r.translation.z = KF.r_N(2);

    t_n2r.rotation.w =  q_n2r(0);
    t_n2r.rotation.x =  q_n2r(1);
    t_n2r.rotation.y =  q_n2r(2);
    t_n2r.rotation.z =  q_n2r(3);

  auto message = interfaces::msg::Kf();
    message.r2c = t_r2c;
    message.n2r = t_n2r;
  
  sensor_msgs::msg::Image::SharedPtr img_msg; 
  img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, KF.frame).toImageMsg();        
  message.img = *img_msg;

  pub_kf_->publish(message);
  //cout << KF.r_N << endl;

}
//------------------------------------------------------------------------------------------------------------------------------
void EKFslam::Handle_pos_update_service(const std::shared_ptr<interfaces::srv::LSposUpdate::Request> request,std::shared_ptr<interfaces::srv::LSposUpdate::Response> response)
{
  
  double delta_x = request->delta_x;
  double delta_y = request->delta_y;
  double delta_z = request->delta_z;
  string type = request->type;
 
  if(delta_x != 0 || delta_y != 0 || delta_z != 0  )
  {
    mutex_pos_update.lock();

      pos_update.delta_pos_update[0] = delta_x;
      pos_update.delta_pos_update[1] = delta_y;
      pos_update.delta_pos_update[2] = delta_z;
      pos_update.pos_update_available = true;
      pos_update.type = type;
    mutex_pos_update.unlock();
  }  

  //cout << "delta_pos update request: " << pos_update << endl;

}



//-----------------------------------------------------------------------------------------------------------------------------

void EKFslam::Handle_ekf_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response) 
    {       
      if (request->cmd == 'r')
      { 
        mutex_ekf_run.lock();
          ekf_steps_loop_run = true; 
        mutex_ekf_run.unlock();  
         
        cout << "localslam-> request received: run ekf loop" << endl;        
        
      } 
      if (request->cmd == 's')
      { 
        mutex_ekf_run.lock();
          ekf_steps_loop_run = false; 
        mutex_ekf_run.unlock();  
         
        cout << "localslam-> request received: stop ekf loop" << endl;        
        
      } 
      if (request->cmd == 'i')
      {
        mutex_init_system.lock();
          re_init_sys = true;
        mutex_init_system.unlock();

        cout << "localslam-> request received: re-initialize ekf " << endl;  
      }



      response->response = true;
    };

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(localslam::EKFslam)