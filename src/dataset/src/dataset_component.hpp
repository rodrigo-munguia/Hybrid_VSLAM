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

#ifndef COMPOSITION__DATASET_COMPONENT_HPP_
#define COMPOSITION__DATASET_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "../../../install/interfaces/include/interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"

#include "parameters.hpp"
#include "getData.hpp"
#include <cv_bridge/cv_bridge.h>

#include "../../localslam/src/localslam_types.hpp"

namespace dataset
{

class DATAset : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit DATAset(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  // declare publishers
  rclcpp::Publisher<interfaces::msg::Alt>::SharedPtr pub_Alt_;
  rclcpp::Publisher<interfaces::msg::Att>::SharedPtr pub_Att_;
  rclcpp::Publisher<interfaces::msg::Gps>::SharedPtr pub_Gps_;
  rclcpp::Publisher<interfaces::msg::Range>::SharedPtr pub_Range_;
  rclcpp::Publisher<interfaces::msg::Frame>::SharedPtr pub_Frame_;
  rclcpp::Publisher<interfaces::msg::Spd>::SharedPtr pub_Spd_;       
  rclcpp::Publisher<interfaces::msg::Ododiff>::SharedPtr pub_OdoD_;
  rclcpp::Publisher<interfaces::msg::Odovw>::SharedPtr pub_OdoV_;
  
  // declare Services
  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv1_;   
  // declare clients
  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_;
  
  // For ros2 message to opencv MAT convertion 
  sensor_msgs::msg::Image::SharedPtr img_msg;

  //------
  rclcpp::TimerBase::SharedPtr timer_;

  
  bool data_run;  // flag for dataset running state

  void setParameters();


  parameters PAR;

  DATA dat;

  

  //void main_loop();

};

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_