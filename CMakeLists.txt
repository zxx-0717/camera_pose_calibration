cmake_minimum_required(VERSION 3.8)
project(camera_pose_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_ros
  sensor_msgs
  angles
)

# camera_pose_calibration library
add_library(camera_pose_calibration_lib SHARED)
target_sources(camera_pose_calibration_lib
  PRIVATE
  src/camera_pose_calibration.cpp
)

target_include_directories(camera_pose_calibration_lib PUBLIC include)
ament_target_dependencies(camera_pose_calibration_lib
  ${dependencies})
rclcpp_components_register_node(camera_pose_calibration_lib
  PLUGIN "camera_pose_calibration::CameraPoseCalibration" 
  EXECUTABLE camera_pose_calibration)

############install libraries
set(libraries_names
  camera_pose_calibration_lib
)
install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

ament_package()
