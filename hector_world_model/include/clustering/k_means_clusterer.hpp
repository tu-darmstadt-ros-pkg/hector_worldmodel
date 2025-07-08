#ifndef HECTOR_WORLD_MODEL_K_MEANS_CLUSTERER_HPP
#define HECTOR_WORLD_MODEL_K_MEANS_CLUSTERER_HPP

#include <clustering/clusterer_base.hpp>

namespace hector_world_model
{

class KMeansClusterer : public DetectionClusterer
{
public:
  KMeansClusterer( std::atomic<int> &latest_marker_id, std::shared_ptr<WorldModel> node );

  ~KMeansClusterer() noexcept { };

private:
  void fit() override;
  void processNewDetections() override;

  void addClusteringData( const ObjectDetection &detection, int associated_center_idx );

  void weightedKMeans( std::vector<Eigen::Vector3d> &data, std::vector<double> &weights,
                       std::vector<Eigen::Vector3d> &centers, std::vector<int> &assignments );

  void processClusteringResults( std::vector<int> const &assignments,
                                 std::vector<double> const &new_confidences,
                                 std::vector<Eigen::Vector3d> const &new_centers );

  void promoteObjectCandidates( std::vector<double> &candidate_confidences,
                                std::vector<std::vector<int>> &center_assingments );

  void removeAssingedDetections( std::vector<int> &associated_detections );

  void removeDetection( int detection_idx );

  // Object candidate attributes
  // std::vector<Eigen::Vector3d> center_locations_;
  // std::vector<std::vector<int>> associated_detections_;

  // Detections attributes
  std::vector<int> assignments_;
  std::vector<Eigen::Vector3d> detection_locations_;
  std::vector<double> detection_confidences_;
};

} // namespace hector_world_model

#endif