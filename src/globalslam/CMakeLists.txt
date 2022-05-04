cmake_minimum_required(VERSION 3.8)
project(globalslam)

#set( CMAKE_MESSAGE_LOG_LEVEL "STATUS" )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -fpermissive -DARMA_DONT_USE_WRAPPER -DARMA_USE_BLAS -DARMA_USE_LAPACK -z defs)
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
find_package(OpenCV REQUIRED OPTIONAL_COMPONENTS opencv_sfm)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(GTSAM REQUIRED)
#find_package(GTSAMCMakeTools)
#include_directories(${GTSAM_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(/usr/include/eigen3)

#target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)
#include_directories(${GTSAM_INCLUDE_DIR})
#find_package(Eigen3)
#find_package(Ceres QUIET REQUIRED)



if(NOT OpenCV_opencv_sfm_FOUND)
  add_definitions("-DNOTHAVE_SFM")
endif()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(SOURCE_FILES 
src/globalslam_component.cpp
src/parameters.cpp
../common/Transforms/quat2R.cpp
../common/Transforms/Ra2b_TO_Quat_a2b.cpp
src/gmap.cpp
src/gmap_init_new_map_points.cpp
../common/Vision/vision.cpp
../common/anms/anms.cpp
src/gmap_visibility_graph_functions.cpp
src/gmap_visual_match.cpp
src/gmap_delete_points.cpp
src/gmap_bundle_adjustment.cpp)

add_library(globalslam_component SHARED ${SOURCE_FILES} )

target_link_libraries(globalslam_component gtsam)

target_compile_definitions(globalslam_component
  PRIVATE "COMPOSITION_BUILDING_DLL")


  ament_target_dependencies(globalslam_component  
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
  "LAPACK"
  "GTSAM"
  "Eigen3"
  "Boost")

  
  
  
rclcpp_components_register_nodes(globalslam_component "globalslam::Gslam")
set(node_plugins "${node_plugins}globalslam::Gslam;$<TARGET_FILE:globalslam_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


install(TARGETS
  globalslam_component  
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

get_target_property(OUT globalslam_component LINK_LIBRARIES)
message(STATUS ${OUT})
message(STATUS "hola")


ament_package()
