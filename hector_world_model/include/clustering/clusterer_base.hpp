#ifndef HECTOR_WORLD_MODEL_CLUSTERER_BASE_HPP
#define HECTOR_WORLD_MODEL_CLUSTERER_BASE_HPP

#include <hector_world_model/object.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace hector_world_model
{
class WorldModel;

class DetectionClusterer
{
public:
  DetectionClusterer( std::atomic<int> &latest_marker_id, const std::shared_ptr<WorldModel> &node );

  virtual ~DetectionClusterer() noexcept = default;

  // Add a new detection to be collected for the next clustering run
  void virtual addDetection( const std::shared_ptr<ObjectDetection> &detection );

  void runClustering()
  {
    processNewDetections();
    fit();
  }

  void reset();

  std::vector<Object> getConfirmedObjects();

protected:
  [[nodiscard]] bool redundancy_criterion( const ObjectDetection &d1,
                                           const ObjectDetection &d2 ) const;

  // Run actual clustering algorithm
  virtual void fit() = 0;
  // Process new detections, add relevant non-redundant detections to clustering pool
  virtual void processNewDetections() = 0;

  bool isRedundant( const ObjectDetection &new_detection );

  [[nodiscard]] bool closeToConfirmedObject( const ObjectDetection &new_detection ) const;

  static void
  pubPointMarker( double x, double y, double z, const int &marker_id, const int &color_idx,
                  const double &size, const bool &is_new,
                  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &marker_pub );
  void pubVisualization( const ObjectDetection &detection, bool is_new,
                         bool was_dismissed = false ) const;
  void pubVisualization( const ObjectCandidate &candidate, bool is_new ) const;
  void pubVisualization( const Object &confirmed_obj, bool is_new ) const;

  double min_object_distance_;
  double confirmation_confidence_threshold_;
  double initial_center_confidence_threshold_;

  double redundancy_endpoint_distance_threshold_;
  double redundancy_endpoint_angle_threshold_;
  double redundancy_distance_threshold_;

  int max_clustering_iterations_;

  std::vector<ObjectDetection> detection_queue_; // Collects all incoming detections
  std::vector<ObjectDetection>
      object_detections_; // Collects all detections not associated with an confirmed object
  std::vector<ObjectCandidate>
      object_candidates_; // Object candidates that don't have sufficient confidence yet
  std::vector<Object> confirmed_objects_; // Confirmed objects

  std::mutex detection_queue_mutex_;
  std::mutex confirmed_objects_mutex_;

  bool new_detections_received_;

  std::atomic<int> &latest_marker_id_;

  std::weak_ptr<WorldModel> node_;
};
} // namespace hector_world_model

#endif
