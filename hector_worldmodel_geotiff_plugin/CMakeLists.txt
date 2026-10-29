cmake_minimum_required(VERSION 3.10)
project(hector_worldmodel_geotiff_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Define a list of dependencies in alphabetical order
set(DEPENDENCIES
    autonomy_manager_msgs
    hector_geotiff_plugin_interface
    hector_worldmodel_msgs
    hector_ros2_utils
    pluginlib
    rcl_interfaces
    rclcpp
    hector_math
    tf2_ros
    tf2_geometry_msgs
    tf2
    tf2_eigen)

find_package(
  Qt6
  COMPONENTS Widgets
  REQUIRED)

foreach(dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(${EIGEN3_INCLUDE_DIRS})

set(HEADERS include/hector_worldmodel_geotiff_plugin/worldmodel_plugin.hpp)

set(SOURCES src/worldmodel_plugin.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Qt6::Widgets
         rclcpp::rclcpp
         pluginlib::pluginlib
         tf2::tf2
         tf2_ros::tf2_ros
         tf2_eigen::tf2_eigen
         tf2_geometry_msgs::tf2_geometry_msgs
         hector_ros2_utils::hector_ros2_utils
         ${autonomy_manager_msgs_TARGETS}
         ${hector_worldmodel_msgs_TARGETS}
         ${rcl_interfaces_TARGETS})

# Packages whose exported target names are not verified here: fall back to
# include dirs / libraries if the namespaced target does not exist.
foreach(pkg hector_math hector_geotiff_plugin_interface)
  if(TARGET ${pkg}::${pkg})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${pkg}::${pkg})
  else()
    target_include_directories(${PROJECT_NAME} PUBLIC ${${pkg}_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${pkg}_LIBRARIES})
  endif()
endforeach()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

# Install the library to the plugins directory
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# Install the plugin XML
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hector_worldmodel_geotiff_plugin
                                         plugins.xml)

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES} Qt6Widgets)

ament_package()
