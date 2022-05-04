cmake_minimum_required(VERSION 3.8)
project(localslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive -DARMA_DONT_USE_WRAPPER -DARMA_USE_BLAS -DARMA_USE_LAPACK)
 # add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Threads REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
#find_package(Armadillo REQUIRED)



include_directories(include)

set(SOURCE_FILES 
src/localslam_component.cpp 
src/sensors_callbacks.cpp 
src/ekf.cpp 
src/ekf_prediction.cpp
../common/Transforms/Euler_to_Ra2b.cpp
../common/Transforms/Ra2b_TO_Quat_a2b.cpp
src/ekf_attitude_update.cpp
../common/Transforms/AngleWrap.cpp
src/ekf_speed_update.cpp
../common/Transforms/quat2R.cpp
src/ekf_altitude_update.cpp
src/ekf_visual_update.cpp
src/ekf_visual_aux_methods.cpp
../common/anms/anms.cpp
../common/Vision/vision.cpp
src/ekf_Jacobians.cpp
src/ekf_visual_init_feats.cpp
src/ekf_visual_match_feats.cpp
src/ekf_visual_delete_feats.cpp
src/ekf_visual_update_WO_validation.cpp
src/ekf_visual_update_With_validation.cpp
src/parameters.cpp)

add_library(localslam_component SHARED ${SOURCE_FILES})

#add_library(localslam_component SHARED src/localslam_component.cpp)

#add_executable(ekfslam_component src/ekfslam_component.cpp)
#target_include_directories(ekfslam_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(ekfslam_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_compile_definitions(localslam_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

#install(TARGETS ekfslam_component
#  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(localslam_component
  "interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "OpenCV"
  "cv_bridge"
  "sensor_msgs"
  "Threads"
  "geometry_msgs"
  "BLAS"
  "LAPACK")
rclcpp_components_register_nodes(localslam_component "localslam::EKFslam")
set(node_plugins "${node_plugins}localslam::EKFslam;$<TARGET_FILE:localslam_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  localslam_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
