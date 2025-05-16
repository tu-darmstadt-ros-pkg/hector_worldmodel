#include "hector_world_model/world_model.hpp"
#include "opencv2/core.hpp"
#include <functional>

namespace hector_world_model
{

WorldModel::WorldModel() : Node( "world_model" ) { setup(); }

void WorldModel::setup()
{
  // subscriber for handling incoming messages
  detection_subscriber_ = create_subscription<hector_perception_msgs::msg::ObjectDetection2D>(
      "~/input", 10,
      std::bind( &WorldModel::detectionReceivedCallback, this, std::placeholders::_1 ) );

  // publisher for publishing outgoing messages
  publisher_ = create_publisher<std_msgs::msg::Int32>( "~/output", 10 );

  this->declare_parameter( "distance_threshhold", 0.05 );
  this->declare_parameter( "confirmation_confidence_threshhold", 1.0 );
  this->declare_parameter( "initial_center_confidence_threshhold", 0.6 );
  this->declare_parameter( "max_clustering_iterations", 5 );
}

void WorldModel::setupNewClusterer( std::string const &class_name )
{
  /* if ( known_classes_.find( class_name ) != known_classes_.end() )
     return; */

  known_classes_.insert( class_name );

  clusterer_.emplace(
      class_name,
      DetectionClusterer(
          this->get_parameter( "distance_threshhold" ).get_value<double>(),
          this->get_parameter( "confirmation_confidence_threshhold" ).get_value<double>(),
          this->get_parameter( "initial_center_confidence_threshhold" ).get_value<double>(),
          this->get_parameter( "max_clustering_iterations" ).get_value<int>(),
          object_detections_[class_name], object_candidates_[class_name],
          confirmed_objects_[class_name], detection_mutex_[class_name],
          candidate_mutex_[class_name], confirmed_object_mutex_[class_name] ) );
}

void WorldModel::detectionReceivedCallback( const hector_perception_msgs::msg::ObjectDetection2D &msg )
{

  // If new class is detected setup new clusterer

  ObjectDetection detected_obj( msg );

  auto ray_projection_client = ray_projection_clients_[msg.header.frame_id];

  std::shared_ptr<image_projection_msgs::srv::ProjectPixelTo3DRay::Request> request =
      std::make_shared<image_projection_msgs::srv::ProjectPixelTo3DRay::Request>();

  request->pixel.header = msg.header;

  auto bb = msg.bounding_box;
  request->pixel.point.y = ( bb.bottom - bb.top ) / 2 + bb.top;
  request->pixel.point.x = ( bb.right - bb.left ) / 2 + bb.left;

  ray_projection_client->async_send_request(
      request,
      [this, detected_obj](
          rclcpp::Client<image_projection_msgs::srv::ProjectPixelTo3DRay>::SharedFuture response ) {
        // distance_clients[detected_obj.label].call();
      } );
  // Calculate pose and covariance
}

} // namespace hector_world_model

int main( int argc, char *argv[] )
{

  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<hector_world_model::WorldModel>() );
  rclcpp::shutdown();

  return 0;
}
