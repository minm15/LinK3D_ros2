cmake_minimum_required(VERSION 3.5)
project(link3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/LinK3D_extractor.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3 OpenCV PCL cv_bridge pcl_conversions sensor_msgs visualization_msgs
)


add_executable(link3d_rosbag src/RosbagMain.cpp)
ament_target_dependencies(link3d_rosbag
  rclcpp sensor_msgs visualization_msgs pcl_conversions PCL cv_bridge
)
target_link_libraries(link3d_rosbag ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} link3d_rosbag
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
