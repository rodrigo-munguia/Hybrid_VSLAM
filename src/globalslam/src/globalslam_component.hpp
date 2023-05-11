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
#include "interfaces/srv/simple_serv.hpp"
#include "globalslam_types.hpp"
#include <mutex>
#include "gmap.hpp"
#include "cloop.hpp"



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
  rclcpp::Subscription<interfaces::msg::Kf>::SharedPtr sub_Kf_cl_;
        
  // declare callbacks
  void Kf_callback(const interfaces::msg::Kf & msg) const;
  void Kf_cl_callback(const interfaces::msg::Kf & msg) ;
  
  // declare Services
  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_globalslam_run_;
  void Handle_globalslam_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response);

  // declare clients
  rclcpp::Client<interfaces::srv::LSposUpdate>::SharedPtr client_local_slam_pos_update_;
  void Send_local_slam_pos_update(arma::vec::fixed<3> &Delta_kf_n, std::string type);
  
  // declare publishers
  rclcpp::Publisher<interfaces::msg::Gmap>::SharedPtr pub_gmap_data_;
  void Publish_gmap_data();

  //------
  // declare functions 
  void setParameters();

  thread gmap_loop_;
  thread cloop_loop_;
  void GMAP_LOOP();
  bool gmap_updated;

  void CLOOP_LOOP();
  bool gmap_thread_loop_run;
  bool cloop_thread_loop_run;
  
  // variables
  GLOBAL_MAP Gmap; // Global map structure


  std::vector<KEYFRAME> Kf_buffer;
  
  KEYFRAME kf_cl_last;
  bool new_kf_cl;

  // parameters
  parameters PAR;
  
  // mutex
  std::mutex mutex_rx_kf;
  std::mutex mutex_rx_kf_cl;
  std::mutex mutex_get_gm;
  std::mutex mutex_send_pos;
  std::mutex mutex_log_gmap;
  std::mutex mutex_log_cloop;

  bool loop_closed_flag;
  bool gmap_get_log_flag;
  bool cloop_get_log_flag;

  void log_data_g(STOREG &data);
  void log_data_c(STOREC &data);
  void mean_std(std::vector<double> &data, double &mean, double &std, double &sum);
  void log_data_to_file(string file_name,std::pair<std::vector<double>,std::vector<double>> &data);


  
  protected:
  
  
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_