cmake_minimum_required(VERSION 3.8)
project(hector_world_model)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(hector_perception_msgs REQUIRED)
find_package(hector_worldmodel_msgs REQUIRED)
find_package(image_projection_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(world_model src/world_model.cpp src/clusterer_base.cpp
                           src/k_means_clusterer.cpp src/db_scan_clusterer.cpp)

target_include_directories(
  world_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)

target_include_directories(
  world_model
  PUBLIC clustering/clusterer_base.hpp clustering/k_means_clusterer.hpp
         clustering/db_scan_clusterer.hpp hector_world_model/object.hpp)

target_link_libraries(world_model Eigen3::Eigen)

ament_target_dependencies(
  world_model
  rclcpp
  std_msgs
  Eigen3
  OpenCV
  hector_worldmodel_msgs
  hector_perception_msgs
  image_projection_msgs
  visualization_msgs)

install(TARGETS world_model DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
