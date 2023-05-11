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

#ifndef COMPOSITION__LOCALSLAM_COMPONENT_HPP_
#define COMPOSITION__LOCALSLAM_COMPONENT_HPP_

#include "visibility_control.h"
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
#include "interfaces/msg/kf.hpp"
#include "interfaces/msg/robotstate.hpp"
#include "localslam_types.hpp"
#include "ekf_types.hpp"
#include <mutex>
#include "parameters.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/l_spos_update.hpp"
#include "interfaces/msg/lslam.hpp"
#include "../../globalslam/src/globalslam_types.hpp"


using namespace std;

namespace localslam
{

class EKFslam : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit EKFslam(const rclcpp::NodeOptions & options);

  ~EKFslam();

private:
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  
  // declare subscribers
  rclcpp::Subscription<interfaces::msg::Alt>::SharedPtr sub_Alt_;
  rclcpp::Subscription<interfaces::msg::Att>::SharedPtr sub_Att_;
  rclcpp::Subscription<interfaces::msg::Gps>::SharedPtr sub_Gps_;
  rclcpp::Subscription<interfaces::msg::Range>::SharedPtr sub_Range_;
  rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;
  rclcpp::Subscription<interfaces::msg::Spd>::SharedPtr sub_Spd_;
  rclcpp::Subscription<interfaces::msg::Ododiff>::SharedPtr sub_OdoD_;
  rclcpp::Subscription<interfaces::msg::Odovw>::SharedPtr sub_OdoV_;


  // declare callbacks
  void Alt_callback(const interfaces::msg::Alt & msg) const;
  void Att_callback(const interfaces::msg::Att & msg) const;
  void Gps_callback(const interfaces::msg::Gps & msg) const;
  void Range_callback(const interfaces::msg::Range & msg) const;
  void Frame_callback(const interfaces::msg::Frame & msg) const;
  void Speed_callback(const interfaces::msg::Spd & msg) const;
  void OdoDiff_callback(const interfaces::msg::Ododiff &msg) const;
  void OdoVW_callback(const interfaces::msg::Odovw &msg) const;
  
  // declare Services
  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_ekf_run_;
  void Handle_ekf_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response);
  rclcpp::Service<interfaces::srv::LSposUpdate>::SharedPtr srv_pos_update_;
  void Handle_pos_update_service(const std::shared_ptr<interfaces::srv::LSposUpdate::Request> request,std::shared_ptr<interfaces::srv::LSposUpdate::Response> response);
  // declare publishers
  rclcpp::Publisher<interfaces::msg::Lslam>::SharedPtr pub_lslam_data_;
  rclcpp::Publisher<interfaces::msg::Kf>::SharedPtr pub_kf_;
  rclcpp::Publisher<interfaces::msg::Kf>::SharedPtr pub_kf_cl_;
  rclcpp::Publisher<interfaces::msg::Robotstate>::SharedPtr pub_robot_state_;
  
  //------
  rclcpp::TimerBase::SharedPtr timer_;
  
  // for
  DATA getData();
  vector<DATA> DataBuffer; 
  void ManageBuffer(DATA &dat);
  std::mutex mutex_dat;


  
  // main loop
  thread ekf_loop_;
  void EKF_LOOP();
  bool ekf_thread_loop_run;
  bool ekf_steps_loop_run;
  std::mutex mutex_ekf_run;  
  std::mutex mutex_get_state;
  std::mutex mutex_init_system;
  std::mutex mutex_pos_update;
  std::mutex mutex_log;
  
    
  
  bool re_init_sys;
  bool new_slam_state_flag;
  bool get_log_flag;
  
  POS_UPDATE pos_update; 
  
  // parameters
  parameters PAR;
  void setParameters();

  void pub_kf(KEYFRAME &KF);
  void pub_kf_cl(KEYFRAME &KF);
  void pub_lslam_Data(LOCALSLAM_DATA &lslam_data);
  void pub_robot_state(arma::vec::fixed<13> &x_r);


  void log_data(STORE &data);
  void mean_std(std::vector<double> &data, double &mean, double &std, double &sum);
  void log_data_to_file(string file_name,std::pair<std::vector<double>,std::vector<double>> &data);

  protected:
  //void on_timer_pub_lslam_data();
  
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_