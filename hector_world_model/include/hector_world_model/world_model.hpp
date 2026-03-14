#ifndef HECTOR_WORLD_MODEL_WORLD_MODEL_HPP
#define HECTOR_WORLD_MODEL_WORLD_MODEL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <hector_perception_msgs/msg/object_detection2_d.hpp>
#include <hector_perception_msgs/msg/object_detection2_d_array.hpp>
#include <hector_worldmodel_msgs/msg/object3_d_detection.hpp>
#include <hector_worldmodel_msgs/srv/get_confirmed_objects.hpp>
#include <hector_worldmodel_msgs/srv/get_distance_to_obstacle.hpp>
#include <image_projection_msgs/srv/project_pixel_to3_d_ray.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <hector_world_model/object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_world_model
{

class DBScanClusterer;

typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3f;

class WorldModel : public rclcpp::Node
{
public:
  WorldModel();
  ~WorldModel();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr candidate_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr confirmed_marker_pub_;

private:
  void setup();

  void declareParameters();

  void detectionCb( const hector_perception_msgs::msg::ObjectDetection2DArray &msg );

  void setupNewClustererIfNeeded( const std::string &class_name );

  void timerCb();

  void pub3DDetection( const ObjectDetection &obj_detection );

  void getConfirmedObjectsCb(
      const hector_worldmodel_msgs::srv::GetConfirmedObjects::Request::SharedPtr request,
      hector_worldmodel_msgs::srv::GetConfirmedObjects::Response::SharedPtr response );

private:
  rclcpp::CallbackGroup::SharedPtr clustering_timer_group_;
  rclcpp::CallbackGroup::SharedPtr detection_cb_group_;

  rclcpp::Service<hector_worldmodel_msgs::srv::GetConfirmedObjects>::SharedPtr get_confirmed_objects_srv_;

  std::map<std::string, std::unique_ptr<DBScanClusterer>> clusterers_;
  std::mutex cluster_mutex_;
  std::vector<std::thread> clustering_threads_;

  rclcpp::TimerBase::SharedPtr clustering_timer_;

  rclcpp::Subscription<hector_perception_msgs::msg::ObjectDetection2DArray>::SharedPtr detection_subscriber_;
  rclcpp::Subscription<hector_worldmodel_msgs::msg::Object3DDetection>::SharedPtr bag_subscriber_;

  rclcpp::Publisher<hector_worldmodel_msgs::msg::Object3DDetection>::SharedPtr detection_publisher_;

  std::map<std::string, rclcpp::Client<image_projection_msgs::srv::ProjectPixelTo3DRay>::SharedPtr>
      ray_projection_clients_;
  rclcpp::Client<hector_worldmodel_msgs::srv::GetDistanceToObstacle>::SharedPtr distance_to_obstacle_client_;

  std::atomic<int> latest_marker_id_{ 0 };

};

} // namespace hector_world_model

#endif // HECTOR_WORLD_MODEL_WORLD_MODEL_HPP
