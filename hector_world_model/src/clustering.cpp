#include "clustering.hpp"

void hector_world_model::DetectionClusterer::handleNewDetection( const ObjectDetection &detection )
{
  double closest_candidate_distance = std::numeric_limits<double>::max();
  ObjectCandidate *closest_candidate = nullptr;
  int closest_center_idx = -1;

  candidate_mutex_.lock();

  int center_idx = 0;
  for ( auto &object_candidate : object_candidates_ ) {
    double candidate_distance =
        ( object_candidate.pose_.translation() - detection.pose_.translation() ).norm();

    // Find closest object candidate to new detection
    if ( candidate_distance < closest_candidate_distance ) {
      closest_candidate_distance = candidate_distance;
      closest_candidate = &object_candidate;
      closest_center_idx = center_idx;
    }
  }

  // If no object candidate is close enough, create a new one if confidence is sufficient
  if ( closest_candidate_distance < distance_threshhold_ ) {
    candidate_mutex_.unlock();
    // Assign detection to closest object candidate
    addNewDetection( detection, closest_center_idx );
  } else {
    if ( detection.confidence_ >= initial_center_confidence_threshhold_ ) {
      object_candidates_.push_back( ObjectCandidate( detection ) );
      addNewDetection( detection, object_candidates_.size() - 1 );
    } else {
      // Dismiss detection
      candidate_mutex_.unlock();
      return;
    }
  }
  candidate_mutex_.unlock();
}

void hector_world_model::DetectionClusterer::addNewDetection( const ObjectDetection &detection,
                                                              int associated_center_idx )
{
  detection_mutex_.lock();

  object_detections_.push_back( detection );
  detection_confidences_.push_back( detection.confidence_ );
  detection_locations_.push_back( detection.pose_.translation() );
  assignments_.push_back( associated_center_idx );

  new_detections_received_ = true;

  detection_mutex_.unlock();
}

void hector_world_model::DetectionClusterer::fit()
{
  if ( !new_detections_received_ )
    return;

  detection_mutex_.lock();
  candidate_mutex_.lock();

  new_detections_received_ = false;

  removeRedudantDetections();

  // Copy current detections and their assinged candidates
  std::vector<Eigen::Vector3d> data = detection_locations_;
  std::vector<double> confidences = detection_confidences_;
  std::vector<int> assignments = assignments_;

  std::vector<Eigen::Vector3d> centers( object_candidates_.size() );

  candidate_mutex_.unlock();
  detection_mutex_.unlock();

  // Run actual clustering algorithm
  weightedKMeans( data, confidences, centers, assignments );

  // Collect object detections assinged to object candidates
  std::vector<std::vector<int>> center_assingments( centers.size() );
  for ( int i = 0; i < assignments.size(); i++ ) {
    center_assingments[assignments[i]].push_back( i );
  }

  // Aggregate confidences of assigned detections
  std::vector<double> center_confidences( center_confidences.size(), 0 );
  aggregateCenterConfidences( assignments, confidences, center_confidences );

  detection_mutex_.lock();
  candidate_mutex_.lock();

  // Write back results to object candidates and new detection assingments
  writeClusteringResults( assignments, center_confidences, centers );

  detection_mutex_.unlock();
  candidate_mutex_.unlock();

  // Check which object candidates have sufficient confidence and remove their assinged detection
  promoteObjectCandidates( center_confidences, center_assingments );
}

void hector_world_model::DetectionClusterer::weightedKMeans( std::vector<Eigen::Vector3d> &data,
                                                             std::vector<double> &weights,
                                                             std::vector<Eigen::Vector3d> &centers,
                                                             std::vector<int> &assignments )
{
  for ( int k = 0; k < max_clustering_iterations_; k++ ) {
    assignClusters( data, centers, assignments );
    updateCenters( data, weights, centers, assignments );
  }
}

void assignClusters( std::vector<Eigen::Vector3d> &data, std::vector<Eigen::Vector3d> &centers,
                     std::vector<int> &assignments )
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

void updateCenters( std::vector<Eigen::Vector3d> &data, std::vector<double> &weights,
                    std::vector<Eigen::Vector3d> &centers, std::vector<int> &assignments )
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
  for ( int i = 0; i < assignment.size(); i++ ) {
    center_confidences[assignment[i]] += confidences[i];
  }

  for ( int i = 0; i < center_confidences.size(); i++ ) {
    // Apply logistic function, i.e normalization
    center_confidences[i] = 1 / ( 1 + exp( -center_confidences[i] ) );
  }
}

void hector_world_model::DetectionClusterer::promoteObjectCandidates(
    std::vector<double> &candidate_confidences, std::vector<std::vector<int>> &center_assingments )
{
  for ( int i = 0; i < candidate_confidences.size(); i++ ) {
    if ( candidate_confidences[i] > 0.5 ) {
      // Move object candidate to confirmed objects
      confirmed_object_mutex_.lock();
      confirmed_objects_.push_back( Object( object_candidates_[i] ) );
      confirmed_object_mutex_.unlock();

      // Remove object detections assigned to candidate
      detection_mutex_.lock();
      removeAssingedDetections( center_assingments[i] );
      detection_mutex_.unlock();

      // Remove object candidate from candidates
      candidate_mutex_.lock();
      rm( object_candidates_, i );
      candidate_mutex_.unlock();
    }
  }
}

void hector_world_model::DetectionClusterer::removeDetection( int detection_idx )
{
  rm( object_detections_, detection_idx );
  rm( detection_locations_, detection_idx );
  rm( detection_confidences_, detection_idx );
  rm( assignments_, detection_idx );
}

// Remove detections that are close to already confirmed objects
void hector_world_model::DetectionClusterer::removeRedudantDetections()
{
  for ( int i = 0; i < object_detections_.size(); i++ ) {
    for ( auto const &confirmed_object : confirmed_objects_ ) {
      // Check if detection corresponds to already confirmed object
      if ( ( confirmed_object.pose_.translation() - object_detections_[i].pose_.translation() ).norm() <
           distance_threshhold_ ) {
        removeDetection( i );
      }
    }
  }
}

void hector_world_model::DetectionClusterer::removeAssingedDetections(
    std::vector<int> &associated_detections )
{
  for ( int i = 0; i < associated_detections.size(); i++ ) {
    removeDetection( associated_detections[i] );
  }
}

// Removal with constant complexity
template<typename T>
void rm( std::vector<T> &vec, int index )
{
  std::swap( vec[index], vec.back() );
  vec.pop_back();
}

void hector_world_model::DetectionClusterer::writeClusteringResults(
    std::vector<int> const &assignments, std::vector<double> const &new_confidences,
    std::vector<Eigen::Vector3d> const &new_centers )
{
  // Write new assignments
  std::copy( assignments.begin(), assignments.end(), assignments_.begin() );

  for ( int i = 0; i < new_confidences.size(), i++ ) {
    object_candidates_[i].pose_.translation() = new_centers[i];
    object_candidates_[i].aggregated_confidence_ = new_confidences[i];
  }
}

void hector_world_model::DetectionClusterer::reset()
{
  detection_mutex_.lock();
  candidate_mutex_.lock();
  confirmed_object_mutex_.lock();

  object_detections_.clear();
  object_candidates_.clear();
  confirmed_objects_.clear();

  confirmed_object_mutex_.unlock();
  candidate_mutex_.unlock();
  detection_mutex_.unlock();
}