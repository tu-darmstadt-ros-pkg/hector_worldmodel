#ifndef HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP
#define HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP

#include <autonomy_manager_msgs/autonomy_manager_msgs/msg/autonomy_mode.h>
#include <autonomy_manager_msgs/autonomy_manager_msgs/msg/detail/autonomy_mode__struct.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <fstream>
#include <hector_geotiff_plugin_interface/geotiff_plugin_interface.hpp>
#include <hector_worldmodel_msgs/srv/get_confirmed_objects.hpp>
#include <iostream>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_worldmodel_geotiff_plugin
{
class WorldmodelPlugin final : public hector_geotiff_plugin_interface::GeotiffPluginInterface
{
public:
  void initialize( const rclcpp::Node::SharedPtr &node ) override;

  std::string getPluginName() override;

  /**
   * @brief Draw function called to draw on the geotiff image.
   *
   * @param geotiff_writer The GeotiffWriterInterface to draw on.
   */
  void draw( std::shared_ptr<hector_geotiff_plugin_interface::GeotiffWriterInterface> geotiff_writer )
      override;
  void drawTypeDependent( const std::string &class_name, const Eigen::Vector2i &geo_coords,
                          QPainter &qp );

  void reset() override;

private:
  void writeToTextfile();

  static std::string autonomyModeToString( const autonomy_manager_msgs::msg::AutonomyMode &mode );

  std::shared_ptr<rclcpp::CallbackGroup> client_group_;
  std::shared_ptr<hector_geotiff_plugin_interface::GeotiffWriterInterface> geotiff_;

  rclcpp::Client<hector_worldmodel_msgs::srv::GetConfirmedObjects>::SharedPtr get_confirmed_objects_client_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  std::vector<std::tuple<std::string, geometry_msgs::msg::PointStamped,
                         builtin_interfaces::msg::Time, autonomy_manager_msgs::msg::AutonomyMode>>
      latest_object_list_;

  std::set<std::string> hazmat_classes_;

  std::mutex mutex_;
};
} // namespace hector_worldmodel_geotiff_plugin

#endif // HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP
