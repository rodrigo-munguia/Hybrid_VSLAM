# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""
import os
from ament_index_python.packages import get_package_share_directory
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description with multiple components."""
    
    configFile = os.path.join(
        os.getcwd(),         
        'config',
        'mobile_params.yaml'
    )
    #configFile = '/home/rodrigo/Dropbox/RESEARCH/RTSLAM/ros_ws/config/params.yaml'
    
    # Load the parameters specific to your ComposableNode
    with open(configFile, 'r') as file:
        dataParams = yaml.safe_load(file)['/data']['ros__parameters']  
        
    with open(configFile, 'r') as file:
        lslamParams = yaml.safe_load(file)['/lslam']['ros__parameters']
    
    with open(configFile, 'r') as file:
        plotParams = yaml.safe_load(file)['/plot']['ros__parameters']
    
    with open(configFile, 'r') as file:
        gslamParams = yaml.safe_load(file)['/gslam']['ros__parameters']        
    
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            #executable='component_container_mt',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dataset',
                    plugin='dataset::DATAset',
                    name='data',
                    parameters= [dataParams]),                    
                 ComposableNode(
                    package='localslam',
                    plugin='localslam::EKFslam',
                    name='EKFslam',
                    parameters= [lslamParams]),
                 ComposableNode(
                    package='plot',
                    plugin='plot::PLOTscene',
                    name='PLOTscene',
                    parameters= [plotParams]),
                ComposableNode(
                    package='globalslam',
                    plugin='globalslam::Gslam',
                    name='Gslam',
                    parameters= [gslamParams]) 
            ],
            output='screen',
    )
    return launch.LaunchDescription([container])


