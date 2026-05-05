#ifndef HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP
#define HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP

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

private:
  void writeToTextfile();

private:
  std::shared_ptr<rclcpp::CallbackGroup> client_group_;
  std::shared_ptr<hector_geotiff_plugin_interface::GeotiffWriterInterface> geotiff_;

  rclcpp::Client<hector_worldmodel_msgs::srv::GetConfirmedObjects>::SharedPtr get_confirmed_objects_client_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  std::vector<std::pair<std::string, geometry_msgs::msg::PointStamped>> latest_object_list_;

  std::set<std::string> hazmat_classes_;
  std::set<std::string> object_classes_;
};

} // namespace hector_worldmodel_geotiff_plugin

#endif // HECTOR_GEOTIFF_WORLDMODEL_PLUGIN_HPP
