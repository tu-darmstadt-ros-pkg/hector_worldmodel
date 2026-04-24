#include "hector_world_model/world_model.hpp"
#include "opencv2/core.hpp"
#include <chrono>
#include <clustering/db_scan_clusterer.hpp>
#include <functional>

namespace hector_world_model
{

WorldModel::WorldModel() : Node( "world_model" ) { setup(); }

WorldModel::~WorldModel()
{
  for ( auto &t : clustering_threads_ ) {
    if ( t.joinable() ) {
      t.join();
    }
  }

  RCLCPP_INFO( this->get_logger(), "All processing threads joined." );
}

void WorldModel::declareParameters()
{

  this->declare_parameter( "distance_threshhold", 0.4 );
  this->declare_parameter( "confirmation_confidence_threshhold", 0.85 );
  this->declare_parameter( "initial_center_confidence_threshhold", 0.6 );
  this->declare_parameter( "max_clustering_iterations", 8 );

  this->declare_parameter( "redundancy_endpoint_distance_threshhold", 0.05 );
  this->declare_parameter( "redundancy_endpoint_angle_threshhold", 10.0 );
  this->declare_parameter( "redundancy_distance_threshhold", 0.2 );

  this->declare_parameter( "min_neighbours", 3 );
  this->declare_parameter( "epsilon", 0.1 );

  this->declare_parameter( "use_bag_detections", false );
}

void WorldModel::setup()
{
  declareParameters();

  clusterers_ = std::map<std::string, std::unique_ptr<DBScanClusterer>>();

  clustering_timer_group_ =
      this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
  detection_cb_group_ = this->create_callback_group( rclcpp::CallbackGroupType::Reentrant );

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = detection_cb_group_;

  bool use_bag_detections = this->get_parameter( "use_bag_detections" ).get_value<bool>();
  if ( !use_bag_detections ) {
    detection_subscriber_ = create_subscription<hector_perception_msgs::msg::ObjectDetection2DArray>(
        std::string( this->get_namespace() ) + "/object_2D_detections", 10,
        std::bind( &WorldModel::detectionCb, this, std::placeholders::_1 ), opts );

    detection_publisher_ = create_publisher<hector_worldmodel_msgs::msg::Object3DDetection>(
        std::string( this->get_namespace() ) + "/object_3D_detections", 20 );
  } else {

    bag_subscriber_ = create_subscription<hector_worldmodel_msgs::msg::Object3DDetection>(
        std::string( this->get_namespace() ) + "/object_3D_detections", 10,
        [this]( const hector_worldmodel_msgs::msg::Object3DDetection &msg ) {
          setupNewClustererIfNeeded( msg.class_name );

          clusterers_.at( msg.class_name )
              ->addDetection( std::make_shared<ObjectDetection>( msg, latest_marker_id_++ ) );
        },
        opts );
  }

  detection_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      std::string( this->get_namespace() ) + "/world_model_detection_markers", 20 );
  candidate_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      std::string( this->get_namespace() ) + "/world_model_candidate_markers", 20 );
  confirmed_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      std::string( this->get_namespace() ) + "/world_model_confirmed_markers", 20 );

  get_confirmed_objects_srv_ = this->create_service<hector_worldmodel_msgs::srv::GetConfirmedObjects>(
      std::string( this->get_namespace() ) + "/get_confirmed_objects",
      std::bind( &WorldModel::getConfirmedObjectsCb, this, std::placeholders::_1,
                 std::placeholders::_2 ) );

  // ray_projection_clients_ = std::map<std::string, rclcpp::Client<image_projection_msgs::srv::ProjectPixelTo3DRay>>();

  distance_to_obstacle_client_ =
      this->create_client<hector_worldmodel_msgs::srv::GetDistanceToObstacle>(
          std::string( this->get_namespace() ) + "/get_distance_to_obstacle", rclcpp::QoS( 10 ),
          detection_cb_group_ );

  if ( !distance_to_obstacle_client_->wait_for_service( std::chrono::seconds( 5 ) ) ) {
    RCLCPP_ERROR( this->get_logger(), "Distance to obstacle service not available after waiting" );
    // Handle failure: maybe exit or retry later
  } else {
    RCLCPP_INFO( this->get_logger(), "Service is available" );
  }

  clustering_timer_ = this->create_timer(
      std::chrono::seconds( 2 ), [this]() { timerCb(); }, clustering_timer_group_ );
  // clustering_timer_->cancel();
}

void WorldModel::setupNewClustererIfNeeded( const std::string &class_name )
{
  cluster_mutex_.lock();
  if ( clusterers_.find( class_name ) == clusterers_.end() ) {
    clusterers_.emplace( class_name, std::make_unique<DBScanClusterer>(
                                         latest_marker_id_, std::static_pointer_cast<WorldModel>(
                                                                shared_from_this() ) ) );
  }
  cluster_mutex_.unlock();
}

void WorldModel::detectionCb( const hector_perception_msgs::msg::ObjectDetection2DArray &msg )
{
  // If camera frame was not encountered before add new client
  if ( ray_projection_clients_.find( msg.header.frame_id ) == ray_projection_clients_.end() ) {

    // /athena/front_wideangle/pinhole_front/image_rect_color or /athena/back_wideangle/pinhole_front/image_rect_color
    bool is_front = msg.header.frame_id.find( "front" ) != std::string::npos;
    std::string service_topic =
        is_front ? "image_projection_pinhole_front" : "image_projection_pinhole_back";

    ray_projection_clients_[msg.header.frame_id] =
        this->create_client<image_projection_msgs::srv::ProjectPixelTo3DRay>(
            std::string( this->get_namespace() ) + "/" + service_topic + "/project_pixel_to_ray",
            rclcpp::QoS( 10 ), detection_cb_group_ );
  }

  const auto ray_projection_client = ray_projection_clients_[msg.header.frame_id];

  for ( const hector_perception_msgs::msg::ObjectDetection2D &single_detection : msg.detections ) {
    auto detected_obj = std::make_shared<ObjectDetection>( single_detection, latest_marker_id_++ );

    auto request = std::make_shared<image_projection_msgs::srv::ProjectPixelTo3DRay::Request>();

    request->pixel.header = msg.header;

    auto bb = single_detection.bounding_box;
    request->pixel.point.y = ( bb.bottom - bb.top ) / 2 + bb.top;
    request->pixel.point.x = ( bb.right - bb.left ) / 2 + bb.left;

    ray_projection_client->async_send_request(
        request,
        [this, detected_obj](
            rclcpp::Client<image_projection_msgs::srv::ProjectPixelTo3DRay>::SharedFuture response ) {
          auto result = response.get();

          auto dist_request =
              std::make_shared<hector_worldmodel_msgs::srv::GetDistanceToObstacle::Request>();

          // Workaround for simulation, image_projection does not use sim_time
          dist_request->point.header = detected_obj->header_;

          RCLCPP_INFO( this->get_logger(), "Dist request time: %u.%u. Current time: %u.%u",
                       dist_request->point.header.stamp.sec, dist_request->point.header.stamp.nanosec,
                       this->now().seconds(), this->now().nanoseconds() );

          dist_request->point.point.x = result->ray.point.x;
          dist_request->point.point.y = result->ray.point.y;
          dist_request->point.point.z = result->ray.point.z;

          /*
          // Check if topic is namespaced
          if ( "/" + frame_name.substr( 0, frame_name.find( "/" ) ) == std::string( this->get_namespace() ) )
            // Remove namespace
            dist_request->point.header.frame_id = frame_name.substr( frame_name.find( "/" ) + 1 );

          Eigen::Matrix3d K;
          K << 360.00051498413086, 0.0, 360.0, 0.0, 360.00051498413086, 240.0, 0.0, 0.0, 1.0;
          // Convert to homogeneous pixel coordinates
          Eigen::Vector3d pixel_coords( request->pixel.point.x, request->pixel.point.y, 1.0 );

          // Convert to camera coordinates (up to scale)
          Eigen::Vector3d camera_coords = K.inverse() * pixel_coords;
          // dist_request->point.point = result->ray.point;

          dist_request->point.point.x = camera_coords.x();
          dist_request->point.point.y = camera_coords.y();
          dist_request->point.point.z = camera_coords.z();*/

          distance_to_obstacle_client_->async_send_request(
              dist_request,
              [this, detected_obj](
                  rclcpp::Client<hector_worldmodel_msgs::srv::GetDistanceToObstacle>::SharedFuture
                      response ) {
                auto result = response.get();

                detected_obj->pose_.translation() =
                    Eigen::Vector3d( result->end_point.point.x, result->end_point.point.y,
                                     result->end_point.point.z );
                detected_obj->direction_ = detected_obj->pose_.translation().normalized();
                detected_obj->distance_ = result->distance;

                detected_obj->projection_support_ = ( 1.0 - exp( -1.0 / ( result->distance ) ) );
                detected_obj->calculateConfidence();

                setupNewClustererIfNeeded( detected_obj->class_name_ );

                clusterers_.at( detected_obj->class_name_ )->addDetection( detected_obj );

                pub3DDetection( *detected_obj );
              } );
        } );
  }
}

void WorldModel::pub3DDetection( const ObjectDetection &obj_detection )
{
  hector_worldmodel_msgs::msg::Object3DDetection msg;
  msg.position.header = obj_detection.header_;
  msg.position.point.x = obj_detection.pose_.translation().x();
  msg.position.point.y = obj_detection.pose_.translation().y();
  msg.position.point.z = obj_detection.pose_.translation().z();
  msg.confidence = obj_detection.confidence_;
  msg.projection_support = obj_detection.projection_support_;
  msg.detection_score = obj_detection.detection_score_;
  msg.class_name = obj_detection.class_name_;

  detection_publisher_->publish( msg );
}

void WorldModel::timerCb()
{
  clustering_timer_->cancel();

  clustering_threads_.clear();
  clustering_threads_.reserve( clusterers_.size() );

  for ( auto &clusterer : clusterers_ ) {
    clustering_threads_.emplace_back(
        std::thread{ [&c = clusterer.second]() { c->runClustering(); } } );
  }
  for ( auto &thread : clustering_threads_ ) thread.join();

  clustering_timer_->reset();
}

void WorldModel::getConfirmedObjectsCb(
    const hector_worldmodel_msgs::srv::GetConfirmedObjects::Request::SharedPtr request,
    hector_worldmodel_msgs::srv::GetConfirmedObjects::Response::SharedPtr response )
{

  for ( auto &cluster_entry : clusterers_ ) {

    for ( const auto &confirmed_object : cluster_entry.second->getConfirmedObjects() ) {

      geometry_msgs::msg::PointStamped position;
      position.header = confirmed_object.header_;
      position.point.x = confirmed_object.pose_.translation().x();
      position.point.y = confirmed_object.pose_.translation().y();
      position.point.z = confirmed_object.pose_.translation().z();

      response->class_names.push_back( cluster_entry.first );
      response->positions.push_back( position );
    }
  }
}

} // namespace hector_world_model

int main( int argc, char *argv[] )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<hector_world_model::WorldModel>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node( node );
  executor.spin();

  rclcpp::shutdown();
  return 0;
}