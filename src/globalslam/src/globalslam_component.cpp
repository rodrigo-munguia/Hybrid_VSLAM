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


using namespace std;

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
  
  // Set services
  
  
  // Set publishers
  

  // Use a timer to schedule periodic message publishing.
  
  
}
//--------------------------------------------------------------------------
Gslam::~Gslam()
{ 
  
}
//-----------------------------------------------------------------------------

void Gslam::setParameters()
{  
   //  Declare node parameters (and default values)
   
 
   

}





}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(globalslam::Gslam)