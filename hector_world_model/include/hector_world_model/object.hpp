#ifndef HECTOR_WORLD_MODEL_OBJECT_HPP
#define HECTOR_WORLD_MODEL_OBJECT_HPP

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include <hector_perception_msgs/msg/object_detection2_d.hpp>
#include <hector_worldmodel_msgs/msg/object3_d_detection.hpp>

namespace hector_world_model
{

typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;

class ObjectDetection
{
public:
  ObjectDetection( const hector_perception_msgs::msg::ObjectDetection2D &msg, int vis_marker_id )
      : header_( msg.header ), class_name_( msg.label ), detection_score_( msg.score ),
        projection_support_( 0.0 ), confidence_( 0.0 ), pose_( Transform3d::Identity() ),
        covariance_( Eigen::Matrix3f::Identity() ), vis_marker_id_( vis_marker_id )
  {
  }

  ObjectDetection( const hector_worldmodel_msgs::msg::Object3DDetection &msg, int vis_marker_id )
      : header_( msg.position.header ), class_name_( msg.class_name ),
        detection_score_( msg.detection_score ), projection_support_( msg.projection_support ),
        confidence_( msg.confidence ), pose_( Transform3d::Identity() ),
        covariance_( Eigen::Matrix3f::Identity() ), vis_marker_id_( vis_marker_id )
  {
    pose_.translation().x() = msg.position.point.x;
    pose_.translation().y() = msg.position.point.y;
    pose_.translation().z() = msg.position.point.z;
  }

  ~ObjectDetection() noexcept { }

  void calculateConfidence() { confidence_ = detection_score_ * projection_support_; }

  std_msgs::msg::Header header_;
  std::string class_name_;
  double detection_score_;
  double projection_support_;
  double confidence_;
  Transform3d pose_;
  Eigen::Matrix3f covariance_;
  int vis_marker_id_;
  Eigen::Vector3d direction_; // Direction vector for the detection
  double distance_;           // Distance to the detection
};

class ObjectCandidate
{
public:
  ObjectCandidate( const ObjectDetection &detection, int vis_marker_id )
      : header_( detection.header_ ), class_name_( detection.class_name_ ),
        aggregated_confidence_( detection.confidence_ ), pose_( detection.pose_ ),
        covariance_( detection.covariance_ ), vis_marker_id_( vis_marker_id )
  {
  }

  ObjectCandidate() { }
  ~ObjectCandidate() noexcept { }

  std_msgs::msg::Header header_;
  std::string class_name_;
  double aggregated_confidence_;
  Transform3d pose_;
  Eigen::Matrix3f covariance_;
  int vis_marker_id_;
};

class Object
{

public:
  Object( const ObjectCandidate &candidate, int vis_marker_id )
      : header_( candidate.header_ ), class_name_( candidate.class_name_ ),
        confidence_( candidate.aggregated_confidence_ ),
        intra_class_id_( class_counts_[candidate.class_name_]++ ), pose_( candidate.pose_ ),
        vis_marker_id_( vis_marker_id )
  {
  }
  ~Object() noexcept { }

  static inline std::map<std::string, int> class_counts_;

  std_msgs::msg::Header header_;
  std::string class_name_;
  double confidence_;
  int intra_class_id_;
  Transform3d pose_;
  int vis_marker_id_;
};

} // namespace hector_world_model

#endif