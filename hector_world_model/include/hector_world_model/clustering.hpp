#ifndef HECTOR_WORLD_MODEL_CLUSTERING_HPP
#define HECTOR_WORLD_MODEL_CLUSTERING_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <hector_world_model/world_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace hector_world_model
{

class DetectionClusterer
{
public:
  DetectionClusterer( double distance_threshhold, double confirmation_confidence_threshhold,
                      double initial_center_confidence_threshhold, int max_clustering_iterations,
                      std::vector<ObjectDetection> &object_detections_,
                      std::vector<ObjectCandidate> &object_candidates,
                      std::vector<Object> &confirmed_objects, std::mutex &detection_mutex,
                      std::mutex &candidate_mutex, std::mutex &confirmed_object_mutex )
      : distance_threshhold_( distance_threshhold ),
        confirmation_confidence_threshhold_( confirmation_confidence_threshhold ),
        max_clustering_iterations_( max_clustering_iterations ),
        object_detections_( object_detections_ ), object_candidates_( object_candidates ),
        confirmed_objects_( confirmed_objects ),
        initial_center_confidence_threshhold_( initial_center_confidence_threshhold ),
        detection_mutex_( detection_mutex ), candidate_mutex_( candidate_mutex ),
        confirmed_object_mutex_( confirmed_object_mutex ) { };

  ~DetectionClusterer() noexcept;

  // Add new detection to the clusterer or dismiss of close to confirmed object
  void handleNewDetection( const ObjectDetection &detection );

  // Run clustering algorithm and promote confirmed object candidates
  void fit();

  std::string class_name_; // Assigned class

private:
  void addNewDetection( const ObjectDetection &detection, int associated_center_idx );

  void weightedKMeans( std::vector<Eigen::Vector3d> &data, std::vector<double> &weights,
                       std::vector<Eigen::Vector3d> &centers, std::vector<int> &assignments );

  void promoteObjectCandidates( std::vector<double> &candidate_confidences,
                                std::vector<std::vector<int>> &center_assingments );

  void writeClusteringResults( std::vector<int> const &assignments,
                               std::vector<double> const &new_confidences,
                               std::vector<Eigen::Vector3d> const &new_centers );

  void removeAssingedDetections( std::vector<int> &associated_detections );

  void removeRedudantDetections();

  void removeDetection( int detection_idx );

  void reset();

  double distance_threshhold_;
  double confirmation_confidence_threshhold_;
  double initial_center_confidence_threshhold_;

  int max_clustering_iterations_ = 5;

  std::vector<ObjectDetection>
      &object_detections_; // Collects all detections not associated with an confirmed object
  std::vector<ObjectCandidate>
      &object_candidates_; // Object candidates that don't have sufficient confidence yet
  std::vector<Object> &confirmed_objects_; // Confirmed objects

  std::mutex &detection_mutex_;
  std::mutex &candidate_mutex_;
  std::mutex &confirmed_object_mutex_;

  // Object candidate attributes
  // std::vector<Eigen::Vector3d> center_locations_;
  // std::vector<std::vector<int>> associated_detections_;

  // Detections attributes
  std::vector<int> assignments_;
  std::vector<Eigen::Vector3d> detection_locations_;
  std::vector<double> detection_confidences_;

  bool new_detections_received_;
};
} // namespace hector_world_model

#endif
