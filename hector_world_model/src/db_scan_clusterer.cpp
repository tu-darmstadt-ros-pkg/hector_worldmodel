#include <clustering/db_scan_clusterer.hpp>
#include <hector_world_model/world_model.hpp>

namespace hector_world_model
{

DBScanClusterer::DBScanClusterer( std::atomic<int> &latest_marker_id,
                                  std::shared_ptr<WorldModel> node )
    : DetectionClusterer( latest_marker_id, node )
{
  min_neighbours_ = node->get_parameter( "min_neighbours" ).get_value<int>();
  epsilon_ = node->get_parameter( "epsilon" ).get_value<double>();
}

void DBScanClusterer::fit()
{
  if ( !new_detections_received_ )
    return;
  new_detections_received_ = false;

  // Write back results to object candidates and new detection assingments
  // writeClusteringResults( assignments, center_confidences, centers );

  int n_clusters = run_db_scan();

  RCLCPP_INFO( node_.lock()->get_logger(), "DBScan found %i clusters", n_clusters );
  object_candidates_.resize( n_clusters );
  processClusteringResults();

  confirmed_objects_.clear();
  promoteObjectCandidates();

  // Check which object candidates have sufficient confidence and remove their assinged detections
}

void DBScanClusterer::processNewDetections()
{
  detection_queue_mutex_.lock();
  std::vector<ObjectDetection> new_detections = detection_queue_;
  detection_queue_.clear();
  detection_queue_mutex_.unlock();

  for ( const auto &detection : new_detections ) {

    if ( isRedundant( detection ) ) {
      if ( detection.confidence_ > object_detections_.front().confidence_ ) {
        object_detections_.front() = detection;
        new_detections_received_ = true;
      } else {
        continue;
      }

    } else {
      object_detections_.insert( object_detections_.begin(), detection );
      pubVisualization( object_detections_.front(), true, false );
      new_detections_received_ = true;
    }
  }

  RCLCPP_INFO( node_.lock()->get_logger(), "Current amount of detections: %i",
               object_detections_.size() );
}

void DBScanClusterer::processClusteringResults()
{
  std::vector<std::vector<size_t>> associated_detections( object_candidates_.size() );
  for ( size_t i = 0; i < clustering_data_.size(); i++ ) {
    const auto &data_point = clustering_data_.at( i );
    if ( data_point.clusterID == UNCLASSIFIED || data_point.clusterID == NOISE ) {
      continue; // Skip unclassified or noise points
    }

    associated_detections[data_point.clusterID - 1].push_back( i );
  }

  for ( size_t i = 0; i < associated_detections.size(); i++ ) {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    double confidence_sum = 0.0;
    std::string class_name;

    auto earliest_discovery_time = node_.lock()->get_clock()->now();
    builtin_interfaces::msg::Time earliest_stamp;

    for ( const size_t &detec_idx : associated_detections.at( i ) ) {
      double detec_confidence = object_detections_.at( detec_idx ).confidence_;
      pos += ( object_detections_.at( detec_idx ).pose_.translation() * detec_confidence );
      confidence_sum += detec_confidence;

      class_name = object_detections_.at( detec_idx ).class_name_;

      auto discovery_time = rclcpp::Time( object_detections_.at( detec_idx ).header_.stamp );
      if ( discovery_time < earliest_discovery_time ) {
        earliest_discovery_time = discovery_time;
        earliest_stamp = object_detections_.at( detec_idx ).header_.stamp;
      }
    }
    auto &candidate = object_candidates_.at( i );
    candidate.pose_.translation() = pos / confidence_sum;
    candidate.aggregated_confidence_ = 1.0 / ( 1 + exp( -confidence_sum ) );
    candidate.class_name_ = class_name;
    candidate.header_.frame_id = "world";
    candidate.header_.stamp = earliest_stamp;
    RCLCPP_INFO( node_.lock()->get_logger(), "Cluster %zu: Center at (%f, %f, %f) with confidence %f",
                 i, candidate.pose_.translation().x(), candidate.pose_.translation().y(),
                 candidate.pose_.translation().z(), candidate.aggregated_confidence_ );
  }
}

void DBScanClusterer::promoteObjectCandidates()
{
  std::lock_guard<std::mutex> lock( confirmed_objects_mutex_ );

  for ( size_t i = 0; i < object_candidates_.size(); i++ ) {
    RCLCPP_INFO( node_.lock()->get_logger(), "Considering promoting candidate with confidence %f",
                 object_candidates_[i].aggregated_confidence_ );
    if ( object_candidates_[i].aggregated_confidence_ > confirmation_confidence_threshhold_ ) {
      // Move object candidate to confirmed objects
      auto new_confirmed_obj = Object( object_candidates_[i], latest_marker_id_++ );
      confirmed_objects_.push_back( new_confirmed_obj );
      pubVisualization( new_confirmed_obj, true );
    }
  }
}

int DBScanClusterer::run_db_scan()
{
  clustering_data_.clear();
  clustering_data_.reserve( object_detections_.size() );

  for ( const auto &data_point : object_detections_ ) {
    clustering_data_.emplace_back( Point{ data_point.pose_.translation(), UNCLASSIFIED } );
  }

  int clusterID = 1;
  for ( auto &point : clustering_data_ ) {
    if ( point.clusterID == UNCLASSIFIED ) {
      if ( expandCluster( point, clusterID ) != FAILURE ) {
        clusterID += 1;
      }
    }
  }

  return clusterID - 1; // Return the number of clusters found
}

int DBScanClusterer::expandCluster( Point &point, int clusterID )
{
  std::vector<int> cluster_seeds = calculateCluster( point );

  if ( cluster_seeds.size() < (size_t)min_neighbours_ ) {
    point.clusterID = NOISE;
    return FAILURE;
  } else {
    int index = 0, index_core_point = 0;

    for ( auto &cluster_seed : cluster_seeds ) {
      clustering_data_.at( cluster_seed ).clusterID = clusterID;

      if ( clustering_data_.at( cluster_seed ).position == point.position ) {
        index_core_point = index;
      }

      ++index;
    }
    cluster_seeds.erase( cluster_seeds.begin() + index_core_point );

    for ( std::vector<int>::size_type i = 0, n = cluster_seeds.size(); i < n; ++i ) {
      std::vector<int> cluster_neighbours =
          calculateCluster( clustering_data_.at( cluster_seeds[i] ) );

      if ( cluster_neighbours.size() < (size_t)min_neighbours_ )
        continue;

      for ( const auto &neighbour_idx : cluster_neighbours ) {

        if ( clustering_data_[neighbour_idx].clusterID == UNCLASSIFIED ||
             clustering_data_[neighbour_idx].clusterID == NOISE ) {

          if ( clustering_data_[neighbour_idx].clusterID == UNCLASSIFIED ) {
            cluster_seeds.push_back( neighbour_idx );
            n = cluster_seeds.size();
          }

          clustering_data_[neighbour_idx].clusterID = clusterID;
        }
      }
    }
    return SUCCESS;
  }
}

std::vector<int> DBScanClusterer::calculateCluster( const Point &source_point )
{
  int index = 0;
  std::vector<int> cluster_index;
  for ( const auto &target_point : clustering_data_ ) {
    if ( isWithinEpsilon( source_point, target_point ) ) {
      cluster_index.push_back( index );
    }
    index++;
  }

  return cluster_index;
}

inline bool DBScanClusterer::isWithinEpsilon( const Point &p1, const Point &p2 )
{
  return d( p1, p2 ) <= epsilon_;
}

inline double DBScanClusterer::d( const Point &p1, const Point &p2 )
{

  return ( p1.position - p2.position ).norm();
}

} // namespace hector_world_model