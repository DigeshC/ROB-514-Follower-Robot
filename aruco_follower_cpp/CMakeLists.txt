cmake_minimum_required(VERSION 3.8)
project(aruco_follower_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Release build by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS} 
)

add_executable(aruco_follower_node src/aruco_follower_node.cpp)

ament_target_dependencies(aruco_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(aruco_follower_node
  ${OpenCV_LIBS}
)

install(TARGETS
  aruco_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
