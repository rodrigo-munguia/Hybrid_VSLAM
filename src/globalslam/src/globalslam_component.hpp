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

#ifndef COMPOSITION__GLOBALSLAM_COMPONENT_HPP_
#define COMPOSITION__GLOBALSLAM_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "parameters.hpp"
#include "interfaces/msg/kf.hpp"
#include "interfaces/msg/gmap.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "globalslam_types.hpp"
#include <mutex>
#include "gmap.hpp"



using namespace std;

namespace globalslam
{

class Gslam : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Gslam(const rclcpp::NodeOptions & options);

  ~Gslam();

private:
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  
  // declare subscribers
  rclcpp::Subscription<interfaces::msg::Kf>::SharedPtr sub_Kf_;
        
  // declare callbacks
  void Kf_callback(const interfaces::msg::Kf & msg) const;
  
  // declare Services

  // declare clients
  rclcpp::Client<interfaces::srv::LSposUpdate>::SharedPtr client_local_slam_pos_update_;
  void Send_local_slam_pos_update(arma::vec::fixed<3> &Delta_kf_n);
  
  // declare publishers
  rclcpp::Publisher<interfaces::msg::Gmap>::SharedPtr pub_gmap_data_;
  void Publish_gmap_data();

  //------
  // declare functions 
  void setParameters();

  thread gmap_loop_;
  void GMAP_LOOP();
  
  // variables
  GLOBAL_MAP Gmap; // Global map structure


  std::vector<KEYFRAME> Kf_buffer;
  bool gmap_thread_loop_run;
  
  // parameters
  parameters PAR;
  
  // mutex
  std::mutex mutex_rx_kf;
  std::mutex mutex_get_gm;


  
  protected:
  
  
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_