#include "clustering/clusterer_base.hpp"
#include "hector_world_model/world_model.hpp"

hector_world_model::DetectionClusterer::DetectionClusterer( std::atomic<int> &latest_marker_id,
                                                            std::shared_ptr<WorldModel> node )
    : latest_marker_id_( latest_marker_id ), node_( node )
{
  const auto node_ptr = node_.lock();
  min_object_distance_ = node_ptr->get_parameter( "distance_threshhold" ).get_value<double>();
  confirmation_confidence_threshhold_ =
      node_ptr->get_parameter( "confirmation_confidence_threshhold" ).get_value<double>();
  initial_center_confidence_threshhold_ =
      node_ptr->get_parameter( "initial_center_confidence_threshhold" ).get_value<double>();
  max_clustering_iterations_ =
      node_ptr->get_parameter( "max_clustering_iterations" ).get_value<int>();

  redundancy_endpoint_distance_threshhold_ =
      node_ptr->get_parameter( "redundancy_endpoint_distance_threshhold" ).get_value<double>();
  redundancy_endpoint_angle_threshhold_ =
      node_ptr->get_parameter( "redundancy_endpoint_angle_threshhold" ).get_value<double>();
  redundancy_distance_threshhold_ =
      node_ptr->get_parameter( "redundancy_distance_threshhold" ).get_value<double>();
}

void hector_world_model::DetectionClusterer::reset()
{
  object_detections_.clear();
  object_candidates_.clear();
  confirmed_objects_.clear();
}

std::vector<hector_world_model::Object> hector_world_model::DetectionClusterer::getConfirmedObjects()
{
  std::lock_guard<std::mutex> lock( confirmed_objects_mutex_ );
  return confirmed_objects_;
}

void hector_world_model::DetectionClusterer::addDetection( std::shared_ptr<ObjectDetection> detection )
{
  detection_queue_mutex_.lock();
  detection_queue_.push_back( *detection );
  detection_queue_mutex_.unlock();
}

bool hector_world_model::DetectionClusterer::closeToConfirmedObject( const ObjectDetection &detection )
{
  for ( const auto &confirmed_obj : confirmed_objects_ ) {
    if ( ( confirmed_obj.pose_.translation() - detection.pose_.translation() ).norm() <
         min_object_distance_ ) {
      return true; // Close to a confirmed object
    }
  }

  return false;
}

void hector_world_model::DetectionClusterer::pubVisualization( const ObjectDetection &detection,
                                                               bool is_new, bool was_dismissed )
{
  if ( was_dismissed ) {
    pubPointMarker( detection.pose_.translation().x(), detection.pose_.translation().y(),
                    detection.pose_.translation().z(), detection.vis_marker_id_, 0, 0.1, is_new,
                    node_.lock()->detection_marker_pub_ );
  } else {
    pubPointMarker( detection.pose_.translation().x(), detection.pose_.translation().y(),
                    detection.pose_.translation().z(), detection.vis_marker_id_, 2, 0.1, is_new,
                    node_.lock()->detection_marker_pub_ );
  }
}

double detectionSimilarity( const hector_world_model::ObjectDetection &d1,
                            const hector_world_model::ObjectDetection &d2 )
{
  // Difference of endpoints  + angle between direction vectors + difference in viewing distance
  return ( d1.pose_.translation() - d2.pose_.translation() ).norm() +
         ( 1 - ( d1.direction_.dot( d2.direction_ ) ) + std::abs( d1.distance_ - d2.distance_ ) );
}

bool hector_world_model::DetectionClusterer::redundancy_criterion(
    const hector_world_model::ObjectDetection &d1, const hector_world_model::ObjectDetection &d2 )
{
  bool endpoint_close = ( d1.pose_.translation() - d2.pose_.translation() ).norm() <
                        redundancy_endpoint_distance_threshhold_;
  bool angle_similar =
      std::abs( acos( std::clamp( d1.direction_.dot( d2.direction_ ), -1.0, 1.0 ) ) ) <
      redundancy_endpoint_angle_threshhold_ * ( M_PI / 180.0 );

  bool distance_similar = std::abs( d1.distance_ - d2.distance_ ) < redundancy_distance_threshhold_;

  return endpoint_close && angle_similar && distance_similar;
}

bool hector_world_model::DetectionClusterer::isRedundant( const ObjectDetection &new_detection )
{
  if ( object_detections_.empty() ) {
    return false; // No existing detections to compare against
  }

  if ( redundancy_criterion( new_detection, object_detections_.front() ) ) {
    return true; // New detection is redundant with the closest existing detection
  }

  std::sort( object_detections_.begin(), object_detections_.end(),
             [&new_detection]( const ObjectDetection &a, const ObjectDetection &b ) {
               return detectionSimilarity( a, new_detection ) >
                      detectionSimilarity( b, new_detection );
             } );

  return redundancy_criterion( new_detection, object_detections_.front() );
}

void hector_world_model::DetectionClusterer::pubVisualization( const ObjectCandidate &candidate,
                                                               bool is_new )
{
  pubPointMarker( candidate.pose_.translation().x(), candidate.pose_.translation().y(),
                  candidate.pose_.translation().z(), candidate.vis_marker_id_, 1, 0.075, is_new,
                  node_.lock()->candidate_marker_pub_ );
}

void hector_world_model::DetectionClusterer::pubVisualization( const Object &confirmed_obj,
                                                               bool is_new )
{
  pubPointMarker( confirmed_obj.pose_.translation().x(), confirmed_obj.pose_.translation().y(),
                  confirmed_obj.pose_.translation().z(), confirmed_obj.vis_marker_id_, 3, 0.09,
                  is_new, node_.lock()->confirmed_marker_pub_ );
}

void hector_world_model::DetectionClusterer::pubPointMarker(
    const double x, const double y, const double z, const int &marker_id, const int &color_idx,
    const double &size, const bool &is_new,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub )
{

  std::vector<std::tuple<float, float, float>> marker_colors_ = {
      { 1.0f, 0.0f, 0.0f }, // Red
      { 0.0f, 1.0f, 0.0f }, // Green
      { 0.0f, 0.0f, 1.0f }, // Blue
      { 1.0f, 1.0f, 0.0f }, // Yellow
      { 1.0f, 0.5f, 0.5f }  // Light Red
  };

  // Create a marker message to visualize the cluster
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = rclcpp::Clock().now();
  // marker.ns = marker_ns_;
  marker.id = marker_id; // Unique ID for the marker
  marker.type = visualization_msgs::msg::Marker::SPHERE;

  if ( is_new )
    marker.action = visualization_msgs::msg::Marker::ADD;
  else
    marker.action = visualization_msgs::msg::Marker::MODIFY;

  marker.lifetime = rclcpp::Duration( 0, 0 );

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  // RCLCPP_INFO( get_logger(), "X scale : %f, Y-scale: %f, Z-scale: %f", scale[0], scale[1], scale[2] );

  auto marker_color = marker_colors_[color_idx];

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  // Set the color of the markers
  marker.color.r = std::get<0>( marker_color );
  marker.color.g = std::get<1>( marker_color );
  marker.color.b = std::get<2>( marker_color );
  marker.color.a = 1.0f; // Fully opaque

  marker_pub->publish( marker );
}
