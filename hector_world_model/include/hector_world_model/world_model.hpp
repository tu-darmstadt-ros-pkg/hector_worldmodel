#ifndef HECTOR_WORLD_MODEL_WORLD_MODEL_HPP
#define HECTOR_WORLD_MODEL_WORLD_MODEL_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <hector_perception_msgs/msg/object_detection2_d.hpp>
#include <hector_world_model/clustering.hpp>
#include <hector_world_model/object.hpp>
#include <image_projection_msgs/srv/project_pixel_to3_d_ray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>

namespace hector_world_model
{

typedef Eigen::Transform<float, 3, Eigen::Affine> Transform3f;

class WorldModel : public rclcpp::Node
{
public:
  WorldModel();
  ~WorldModel() noexcept;

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

  void detectionReceivedCallback( const hector_perception_msgs::msg::ObjectDetection2D &msg );

  void setupNewClusterer( std::string const &class_name );

private:
  std::map<std::string, std::mutex> detection_mutex_;
  std::map<std::string, std::mutex> candidate_mutex_;
  std::map<std::string, std::mutex> confirmed_object_mutex_;

  std::map<std::string, std::vector<ObjectDetection>>
      object_detections_; // Collects all detections not associated with an confirmed object
  std::map<std::string, std::vector<ObjectCandidate>>
      object_candidates_; // Object candidates that don't have sufficient confidence yet
  std::map<std::string, std::vector<Object>> confirmed_objects_; // Confirmed objects

  std::map<std::string, rclcpp::Client<image_projection_msgs::srv::ProjectPixelTo3DRay>::SharedPtr>
      ray_projection_clients_;
  // std::map<std::string, <rclcpp::Client<>::SharedPtr> distance_clients_;

  rclcpp::Subscription<hector_perception_msgs::msg::ObjectDetection2D>::SharedPtr detection_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  std::map<std::string, DetectionClusterer> clusterer_;

  double param_ = 1.0;

  std::set<std::string> known_classes_;
};

} // namespace hector_world_model

#endif // HECTOR_WORLD_MODEL_WORLD_MODEL_HPP
