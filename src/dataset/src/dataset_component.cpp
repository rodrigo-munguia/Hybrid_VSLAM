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

#include "dataset_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cinttypes>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "getData.hpp"
#include <cv_bridge/cv_bridge.h>



using namespace std::chrono_literals;
using namespace std;

namespace dataset
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
DATAset::DATAset(const rclcpp::NodeOptions & options)
: Node("data", options), count_(0)
{
  setParameters();
  
  data_run = false;  

  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  //pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  pub_Alt_ = create_publisher<interfaces::msg::Alt>("Altitude_topic",10);
  pub_Att_ = create_publisher<interfaces::msg::Att>("Attitude_topic",10);
  pub_Gps_ = create_publisher<interfaces::msg::Gps>("Gps_topic",10);
  pub_Range_ = create_publisher<interfaces::msg::Range>("Range_topic",10);
  pub_Frame_ = create_publisher<interfaces::msg::Frame>("Frame_topic",10); 
  pub_Spd_ = create_publisher<interfaces::msg::Spd>("Speed_topic",10);
  pub_OdoD_ = create_publisher<interfaces::msg::Ododiff>("OdometryD_topic",10);
  pub_OdoV_ = create_publisher<interfaces::msg::Odovw>("OdometryV_topic",10);      
 
 

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(1ms, std::bind(&DATAset::on_timer, this));
  
  client_ekf_run_ = this->create_client<interfaces::srv::SimpleServ>("ekf_run_service");

  auto handle_simple_service =
    [this](
    const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,
    std::shared_ptr<interfaces::srv::SimpleServ::Response> response
    ) -> void
    { 
      /*
      RCLCPP_INFO(
        this->get_logger(), "Incoming request: [cmd: %" PRId64 "]",
        request->cmd);
      std::flush(std::cout);
      */
      if (request->cmd == 'p')  // play / pause data
      { 
        if (data_run == false)
        {
           data_run = true;
           cout << "dataset -> running data" << endl;
        }
        else
        {
          data_run = false;
          cout << "dataset-> pause data" << endl;
        }  
      }
      if (request->cmd == 'r')  // reset data set
      {
          cout << "dataset-> restart data set" << endl;
          PAR.restart = true; // tell getDataB(PAR) function to start from the beggining
          
          
          auto request_2 = std::make_shared<interfaces::srv::SimpleServ::Request>();
          
          // run local slam
             request_2->cmd = 'r';           
          client_ekf_run_->async_send_request(request_2);    

          // re initialice    
              request_2->cmd = 'i';           
          client_ekf_run_->async_send_request(request_2);
      }  


      response->response = true;
    };


  srv1_ = create_service<interfaces::srv::SimpleServ>("dataset_service", handle_simple_service);

  


}

//----------------------------------------------------------------------------
void DATAset::setParameters()
{  
   //  Declare node parameters (and default values)
   this->declare_parameter<std::string>("Dataset", "bebop");
   this->declare_parameter<std::string>("Dataset_path", "/home/rodrigo/RESEARCH/DataSets/");
   this->declare_parameter<long int>("init_time",0000000000);
   this->declare_parameter<long int>("end_time" ,0010000000);
   this->declare_parameter<double>("run_time",10);
   this->declare_parameter<double>("x_vel_run_time",1); 
  
   // Set parameter struct
   this->get_parameter("Dataset",PAR.Dataset);
   this->get_parameter("Dataset_path", PAR.Dataset_path);
   this->get_parameter("run_time", PAR.run_time);
   this->get_parameter("x_vel_run_time",PAR.x_vel_run_time);
   this->get_parameter("init_time",PAR.init_time);
   this->get_parameter("end_time",PAR.end_time);
   
    

}



//-------------------------------------------------------------------------------------

void DATAset::on_timer()
{
  

  if (data_run == true)
  { 

    if(PAR.Dataset == "bebop")
    {
      dat =  getDataB(PAR);
    }
    else if(PAR.Dataset == "rawseeds")
    {
      dat = getDataR(PAR);
    }
    else if(PAR.Dataset == "loris")
    {
      dat = getDataL(PAR);
    }
    else if(PAR.Dataset == "mobilerobot")
    {
      dat = getDataM(PAR);
    }  
    
    if (dat.data_type != "")
    {
      //cout << dat.data_type << endl;
      if(dat.data_type == "frame")
      {
        auto message = interfaces::msg::Frame();
        message.range = dat.frame.range;      
        img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, dat.frame.image).toImageMsg();        
        message.img = *img_msg;
        message.time = dat.frame.time; 
        pub_Frame_->publish(message);

        //cout << "f" << endl;
      
      }
      if(dat.data_type == "gps")
      {
        auto message = interfaces::msg::Gps();
        message.lat = dat.gps.lat;
        message.lon = dat.gps.lon;
        message.alt = dat.gps.alt;
        message.sat = dat.gps.sat;
        message.time = dat.gps.time;
        pub_Gps_->publish(message);

      }
      if(dat.data_type == "alt")
      {
        auto message = interfaces::msg::Alt();
        message.altitude = dat.alt.altitude;
        message.time = dat.alt.time;
        pub_Alt_->publish(message);

      }
      if(dat.data_type == "att")
      {
        auto message = interfaces::msg::Att();
        message.roll = dat.att.roll;
        message.pitch = dat.att.pitch;
        message.yaw = dat.att.yaw;
        message.time = dat.att.time;        
        pub_Att_->publish(message);

      }
      if(dat.data_type == "range")
      {
        auto message = interfaces::msg::Range();
        message.range = dat.range.range;
        message.volt = dat.range.volt; 
        message.time = dat.range.time;            
        pub_Range_->publish(message);

      }
      if(dat.data_type == "spd")
      {
        auto message = interfaces::msg::Spd();
        message.speed_x = dat.spd.speedX;
        message.speed_y = dat.spd.speedY;
        message.speed_z = dat.spd.speedZ; 
        message.time = dat.spd.time;            
        pub_Spd_->publish(message);

      }
      if(dat.data_type == "odod")
      {
        auto message = interfaces::msg::Ododiff();
        message.ticks_right = dat.odod.TicksRight;
        message.ticks_left = dat.odod.TicksLeft;         
        message.time = dat.odod.time;            
        pub_OdoD_->publish(message);

        //cout << "o" << endl;

      }
      if(dat.data_type == "odov")
      {
        auto message = interfaces::msg::Odovw();
        message.linear_vel = dat.odov.linear_vel;
        message.angular_vel = dat.odov.angular_vel;
        message.time = dat.odov.time;
        pub_OdoV_->publish(message);

      }
      if(dat.data_type == "NULL")
      {
        cout << "dataset-> no more data, for restart dataset press 'r' " << endl;
       // there is no more data to send
       // pause local SLAM component
       auto request_2 = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request_2->cmd = 's';           
        auto result_2 = client_ekf_run_->async_send_request(request_2);

        data_run = false;

      }  


    }  
    /*
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    std::flush(std::cout);

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg));
    cout << PAR.run_time << endl;
    */
  }  
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(dataset::DATAset)