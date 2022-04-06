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
        
  // declare callbacks
  
  
  // declare Services
  
  // declare publishers
  
  //------
  // declare functions 
  void setParameters();
  
  // for
  
  
  // main loop
  
  

  
  // parameters
  parameters PAR;
  


  protected:
  
  
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_