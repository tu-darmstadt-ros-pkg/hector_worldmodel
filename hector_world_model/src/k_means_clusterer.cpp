#include <clustering/k_means_clusterer.hpp>
#include <hector_world_model/world_model.hpp>

// Removal with constant complexity
template<typename T>
void rm( std::vector<T> &vec, int index )
{
  std::swap( vec.at( index ), vec.back() );
  vec.pop_back();
}

void assignClusters( const std::vector<Eigen::Vector3d> &data,
                     const std::vector<Eigen::Vector3d> &centers, std::vector<int> &assignments )
{
  for ( size_t i = 0; i < data.size(); ++i ) {
    double min_distance = std::numeric_limits<double>::max();
    int closest_center = -1;

    for ( size_t j = 0; j < centers.size(); ++j ) {
      double distance = ( data[i] - centers[j] ).norm();
      if ( distance < min_distance ) {
        min_distance = distance;
        closest_center = j;
      }
    }

    assignments[i] = closest_center;
  }
}

void updateCenters( const std::vector<Eigen::Vector3d> &data, const std::vector<double> &weights,
                    std::vector<Eigen::Vector3d> &centers, const std::vector<int> &assignments )
{
  std::vector<double> total_weight( centers.size(), 0 );
  centers.assign( centers.size(), Eigen::Vector3d::Zero() );

  for ( size_t i = 0; i < data.size(); ++i ) {
    centers[assignments[i]] += data[i] * weights[i];
    total_weight[assignments[i]] += weights[i];
  }

  for ( size_t j = 0; j < centers.size(); ++j ) {
    if ( total_weight[j] > 0 ) {
      centers[j] /= total_weight[j];
    }
  }
}

void aggregateCenterConfidences( std::vector<int> const &assignment,
                                 std::vector<double> const &confidences,
                                 std::vector<double> &center_confidences )
{
  for ( size_t i = 0; i < assignment.size(); i++ ) {
    center_confidences[assignment[i]] += confidences[i];
  }

  for ( size_t i = 0; i < center_confidences.size(); i++ ) {
    // Apply logistic function, i.e normalization
    center_confidences[i] = 1 / ( 1 + exp( -center_confidences[i] ) );
  }
}

hector_world_model::KMeansClusterer::KMeansClusterer( std::atomic<int> &latest_marker_id,
                                                      const std::shared_ptr<WorldModel> &node )
    : DetectionClusterer( latest_marker_id, node )
{
}

void hector_world_model::KMeansClusterer::processNewDetections()
{
  std::vector<ObjectDetection> new_detections;
  detection_queue_mutex_.lock();
  new_detections.reserve( detection_queue_.size() );

  new_detections.insert( new_detections.begin(), detection_queue_.begin(), detection_queue_.end() );
  detection_queue_mutex_.unlock();

  for ( const auto &detection : new_detections ) {

    if ( isRedundant( detection ) ) {
      if ( detection.confidence_ > object_detections_.front().confidence_ ) {
        object_detections_.front() = detection;
      } else {
        continue;
      }

    } else {
      pubVisualization( detection, true, true );
      object_detections_.insert( object_detections_.begin(), detection );
    }

    // std::sort( detection_queue_c.begin(), detection_queue_.end(), []( int a, int b ) { return a > b; } );

    pubVisualization( detection, true );

    bool is_redundant = false;
    for ( const auto &confirmed_obj : confirmed_objects_ ) {
      if ( ( confirmed_obj.pose_.translation() - detection.pose_.translation() ).norm() <
           min_object_distance_ ) {
        is_redundant = true;
      }
    }

    if ( is_redundant )
      continue;

    double closest_candidate_distance = std::numeric_limits<double>::max();
    int closest_center_idx = -1;

    int center_idx = 0;
    for ( auto &object_candidate : object_candidates_ ) {
      const double candidate_distance =
          ( object_candidate.pose_.translation() - detection.pose_.translation() ).norm();

      // Find closest object candidate to new detection
      if ( candidate_distance < closest_candidate_distance ) {
        closest_candidate_distance = candidate_distance;
        closest_center_idx = center_idx;
      }
    }

    // If no object candidate is close enough, create a new one if confidence is sufficient
    if ( closest_candidate_distance < min_object_distance_ ) {
      // Assign detection to closest object candidate
      addClusteringData( detection, closest_center_idx );
      // pubVisualization( detection, true );
    } else {
      if ( detection.confidence_ >= initial_center_confidence_threshold_ ) {
        auto candidate = ObjectCandidate( detection, detection.vis_marker_id_ );
        object_candidates_.push_back( candidate );
        addClusteringData( detection, object_candidates_.size() - 1 );
        // pubVisualization( candidate, true );
      }

      else {
        // pubVisualization( detection, true, true );
      }
    }
  }
}

void hector_world_model::KMeansClusterer::addClusteringData( const ObjectDetection &detection,
                                                             int associated_center_idx )
{
  object_detections_.push_back( detection );
  detection_confidences_.push_back( detection.confidence_ );
  detection_locations_.push_back( detection.pose_.translation() );
  assignments_.push_back( associated_center_idx );

  new_detections_received_ = true;
}

void hector_world_model::KMeansClusterer::fit()
{
  if ( !new_detections_received_ )
    return;

  new_detections_received_ = false;

  // Copy current detections and their assigned candidates
  std::vector<Eigen::Vector3d> data = detection_locations_;
  std::vector<double> confidences = detection_confidences_;
  std::vector<int> assignments = assignments_;

  std::vector<Eigen::Vector3d> centers( object_candidates_.size() );

  // Run actual clustering algorithm
  weightedKMeans( data, confidences, centers, assignments );

  // Collect object detections assigned to object candidates
  std::vector<std::vector<int>> center_assignments( centers.size() );
  for ( size_t i = 0; i < assignments.size(); i++ ) {
    center_assignments.at( assignments[i] ).push_back( i );
  }

  // Aggregate confidences of assigned detections
  std::vector<double> center_confidences( centers.size(), 0.0 );
  aggregateCenterConfidences( assignments, confidences, center_confidences );

  // Write back results to object candidates and new detection assignments
  processClusteringResults( assignments, center_confidences, centers );

  // Check which object candidates have sufficient confidence and remove their assigned detections
  promoteObjectCandidates( center_confidences, center_assignments );
}

void hector_world_model::KMeansClusterer::weightedKMeans( const std::vector<Eigen::Vector3d> &data,
                                                          const std::vector<double> &weights,
                                                          std::vector<Eigen::Vector3d> &centers,
                                                          std::vector<int> &assignments ) const
{
  for ( int k = 0; k < max_clustering_iterations_; k++ ) {
    assignClusters( data, centers, assignments );
    updateCenters( data, weights, centers, assignments );
  }
}

void hector_world_model::KMeansClusterer::promoteObjectCandidates(
    const std::vector<double> &candidate_confidences,
    const std::vector<std::vector<int>> &center_assignments )
{
  for ( size_t i = 0; i < candidate_confidences.size(); i++ ) {
    if ( candidate_confidences[i] > 0.5 ) {
      // Move object candidate to confirmed objects

      auto new_confirmed_obj = Object( object_candidates_[i], latest_marker_id_++ );
      confirmed_objects_.push_back( new_confirmed_obj );
      pubVisualization( new_confirmed_obj, true );

      // Remove object detections assigned to candidate
      removeAssignedDetections( center_assignments[i] );

      // Remove object candidate from candidates
      rm( object_candidates_, i );
    }
  }
}

void hector_world_model::KMeansClusterer::removeDetection( const int detection_idx )
{
  rm( object_detections_, detection_idx );
  rm( detection_locations_, detection_idx );
  rm( detection_confidences_, detection_idx );
  rm( assignments_, detection_idx );
}

void hector_world_model::KMeansClusterer::removeAssignedDetections(
    const std::vector<int> &associated_detections )
{
  for ( size_t i = 0; i < associated_detections.size(); i++ ) {
    removeDetection( associated_detections[i] - i );
  }
}

void hector_world_model::KMeansClusterer::processClusteringResults(
    std::vector<int> const &assignments, std::vector<double> const &new_confidences,
    std::vector<Eigen::Vector3d> const &new_centers )
{
  // Write new assignments
  std::copy( assignments.begin(), assignments.end(), assignments_.begin() );

  for ( size_t i = 0; i < new_confidences.size(); i++ ) {
    object_candidates_[i].pose_.translation() = new_centers[i];
    object_candidates_[i].aggregated_confidence_ = new_confidences[i];
  }
}