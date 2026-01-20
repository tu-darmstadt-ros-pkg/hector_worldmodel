#include <QBrush>
#include <QPainter>
#include <hector_math/helpers/coloring.h>
#include <hector_worldmodel_geotiff_plugin/worldmodel_plugin.hpp>

namespace hector_worldmodel_geotiff_plugin
{

void WorldmodelPlugin::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  client_group_ = node_->create_callback_group( rclcpp::CallbackGroupType::Reentrant );

  get_confirmed_objects_client_ =
      node_->create_client<hector_worldmodel_msgs::srv::GetConfirmedObjects>(
          std::string( node_->get_namespace() ) + "/get_confirmed_objects", rclcpp::QoS( 10 ),
          client_group_ );

  hazmat_classes_ = { "non_flammable_gas", "organic_peroxide" };

  update_timer_ = node_->create_wall_timer(
      std::chrono::seconds( 1 ),
      [this]() {
        try {
          auto request =
              std::make_shared<hector_worldmodel_msgs::srv::GetConfirmedObjects::Request>();
          get_confirmed_objects_client_->async_send_request(
              request,
              [this]( rclcpp::Client<hector_worldmodel_msgs::srv::GetConfirmedObjects>::SharedFuture
                          response ) {
                auto result = response.get();
                latest_object_list_.clear();
                for ( size_t i = 0; i < result->class_names.size(); i++ ) {
                  const auto &class_name = result->class_names.at( i );
                  const auto &position = result->positions.at( i );

                  latest_object_list_.emplace_back( class_name, position );
                }
              } );

        } catch ( const std::exception &e ) {
          RCLCPP_WARN( node_->get_logger(), "Error while drawing confirmed objects: %s", e.what() );
        }
      },
      client_group_ );
}

std::string WorldmodelPlugin::getPluginName() { return "worldmodel_plugin"; }

void WorldmodelPlugin::draw(
    std::shared_ptr<hector_geotiff_plugin_interface::GeotiffWriterInterface> geotiff )
{
  RCLCPP_INFO_STREAM( node_->get_logger(), "Drawing Plugin: " << getPluginName() );
  geotiff_ = geotiff;
  QPainter qp = QPainter( &geotiff_->getImage() );

  for ( const auto &confirmed_object : latest_object_list_ ) {
    const auto &class_name = confirmed_object.first;

    const auto coords =
        Eigen::Vector2f{ confirmed_object.second.point.x, confirmed_object.second.point.y };

    Eigen::Vector2i geo_coords = geotiff_->transformWorldToGeoCoords( coords );

    drawTypeDependent( class_name, geo_coords, qp );
  }
}

void WorldmodelPlugin::drawTypeDependent( const std::string &class_name,
                                          const Eigen::Vector2i &geo_coords, QPainter &qp )
{
  if ( hazmat_classes_.find( class_name ) != hazmat_classes_.end() ) {
    geotiff_->drawObjectOfInterest( qp, geo_coords, class_name.substr( 0, 2 ), { 255, 100, 30 },
                                    { 255, 255, 255 }, Eigen::Vector2f( 1.0f, 1.0f ),
                                    hector_geotiff_plugin_interface::Shape::SHAPE_DIAMOND, true,
                                    true );
    return;
  }
  if ( object_classes_.find( class_name ) != object_classes_.end() ) {
    geotiff_->drawObjectOfInterest( qp, geo_coords, class_name.substr( 0, 2 ), { 240, 10, 10 },
                                    { 255, 255, 255 }, Eigen::Vector2f( 1.0f, 1.0f ),
                                    hector_geotiff_plugin_interface::Shape::SHAPE_DIAMOND, true,
                                    true );
    return;
  }
  RCLCPP_WARN( node_->get_logger(), "Unknown class name: %s", class_name.c_str() );
  writeToTextfile();
}

void WorldmodelPlugin::writeToTextfile()
{
  const std::string filename = "~/hector/RoboCup2025-Hector-Labyrinth-SemiFinals-17:00-pois.csv";

  // Open the file in output mode with truncation
  std::ofstream file( filename, std::ios::out | std::ios::trunc );

  if ( !file ) {
    RCLCPP_WARN( node_->get_logger(), "Could not open text file %s to save maze objects",
                 filename.c_str() );
    return;
  }

  // Write to the file
  file << "pois\n";
  file << "1.3\n";
  file << "Hector\n";
  file << "Germany\n";
  file << "2025-07-19\n";
  file << "17:00\n";
  file << "SemiFinals\n";

  int idx = 0;
  for ( const auto &confirmed_object : latest_object_list_ ) {
    file << idx++ << ", ";

    rclcpp::Time stamp( confirmed_object.second.header.stamp );
    std::time_t time_t_stamp = static_cast<time_t>( stamp.seconds() );
    file << std::put_time( std::gmtime( &time_t_stamp ), "%H:%M:%S" ) << ", ";

    const auto pos = confirmed_object.second.point;
    file << pos.x << ", ";
    file << pos.y << ", ";
    file << pos.z << ", ";
    file << node_->get_namespace() << ", ";
    file << "exploration" << "\n";
  }
}

} // namespace hector_worldmodel_geotiff_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_worldmodel_geotiff_plugin::WorldmodelPlugin,
                        hector_geotiff_plugin_interface::GeotiffFunctionPlugin )
