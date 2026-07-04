#include <QBrush>
#include <QFontMetrics>
#include <QPainter>
#include <cmath>
#include <hector_math/helpers/coloring.h>
#include <hector_worldmodel_geotiff_plugin/worldmodel_plugin.hpp>
#include <iomanip>

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
                const auto &result = response.get();
                std::lock_guard<std::mutex> lock( mutex_ );
                latest_object_list_.clear();
                for ( size_t i = 0; i < result->class_names.size(); i++ ) {
                  const auto &class_name = result->class_names.at( i );
                  const auto &position = result->positions.at( i );
                  const auto &detection_time = result->detection_times.at( i );
                  const auto &operation_mode = result->operation_modes.at( i );

                  latest_object_list_.emplace_back( class_name, position, detection_time,
                                                    operation_mode );
                }
              } );
        } catch ( const std::exception &e ) {
          RCLCPP_WARN( node_->get_logger(), "Error while drawing confirmed objects: %s", e.what() );
        }
      },
      client_group_ );
}

std::string WorldmodelPlugin::getPluginName() { return "worldmodel_plugin"; }

void WorldmodelPlugin::reset()
{
  std::lock_guard<std::mutex> lock( mutex_ );
  latest_object_list_.clear();
}

void WorldmodelPlugin::draw(
    const std::shared_ptr<hector_geotiff_plugin_interface::GeotiffWriterInterface> geotiff_writer )
{
  RCLCPP_INFO_STREAM( node_->get_logger(), "Drawing Plugin: " << getPluginName() );
  geotiff_ = geotiff_writer;

  std::lock_guard<std::mutex> lock( mutex_ );

  // Assign each object the same id used to label its marker on the map, its legend row,
  // and its row in the exported CSV (see writeToTextfile), so all three can be cross-referenced.
  std::vector<std::pair<int, std::string>> legend_entries;
  legend_entries.reserve( latest_object_list_.size() );
  int next_id = 0;
  for ( const auto &[class_name, point, detection_time, operation_mode] : latest_object_list_ ) {
    legend_entries.emplace_back( next_id++, class_name );
  }

  const Eigen::Vector2i legend_origin( geotiff_->getImage().width(), 0 );
  if ( !legend_entries.empty() ) {
    const float pixels_per_meter = geotiff_->getPixelsPerGeotiffMeter();
    geotiff_->setFont( 6 );
    const QFontMetrics font_metrics( geotiff_->getMapDrawFont() );
    int max_text_width = 0;
    for ( const auto &[id, class_name] : legend_entries ) {
      max_text_width = std::max(
          max_text_width, font_metrics.horizontalAdvance( QString::fromStdString( class_name ) ) );
    }
    const int margin = static_cast<int>( pixels_per_meter * 0.2f );
    const int swatch_size = static_cast<int>( pixels_per_meter * 0.5f );
    const int legend_width_px = 3 * margin + swatch_size + max_text_width;
    const int legend_width_m =
        std::max( 1, static_cast<int>( std::ceil( legend_width_px / pixels_per_meter ) ) );
    geotiff_->extendImage( hector_geotiff_plugin_interface::Direction::RIGHT, legend_width_m,
                           geotiff_->getImage() );
  }

  auto qp = QPainter( &geotiff_->getImage() );
  int id = 0;
  for ( const auto &[class_name, point, detection_time, operation_mode] : latest_object_list_ ) {
    const auto coords = Eigen::Vector2f{ point.point.x, point.point.y };
    Eigen::Vector2i geo_coords = geotiff_->transformWorldToGeoCoords( coords );
    qp.save();
    drawTypeDependent( class_name, geo_coords, qp, id++ );
    qp.restore();
  }

  if ( !legend_entries.empty() ) {
    drawLegend( qp, legend_origin, legend_entries );
  }

  if ( !latest_object_list_.empty() ) {
    writeToTextfile();
  }
  RCLCPP_INFO_STREAM( node_->get_logger(), "Drawn Plugin: " << getPluginName() );
}

QColor WorldmodelPlugin::getClassColor( const std::string &class_name ) const
{
  if ( hazmat_classes_.find( class_name ) != hazmat_classes_.end() ) {
    return { 255, 100, 30 };
  }
  return { 240, 10, 10 };
}

void WorldmodelPlugin::drawTypeDependent( const std::string &class_name,
                                          const Eigen::Vector2i &geo_coords, QPainter &qp, int id )
{
  geotiff_->drawObjectOfInterest( qp, geo_coords, std::to_string( id ), getClassColor( class_name ),
                                  { 255, 255, 255 }, Eigen::Vector2f( 1.0f, 1.0f ),
                                  hector_geotiff_plugin_interface::Shape::SHAPE_DIAMOND, true, true );
}

void WorldmodelPlugin::drawLegend( QPainter &qp, const Eigen::Vector2i &origin,
                                   const std::vector<std::pair<int, std::string>> &entries )
{
  const float pixels_per_meter = geotiff_->getPixelsPerGeotiffMeter();
  const int margin = static_cast<int>( pixels_per_meter * 0.2f );
  const int row_height = static_cast<int>( pixels_per_meter * 0.5f );

  qp.save();
  geotiff_->setFont( 6 );
  qp.setFont( geotiff_->getMapDrawFont() );
  qp.setPen( QColor( 0, 0, 0 ) );
  qp.drawText( origin.x() + margin, origin.y() + row_height, "Legend" );

  int row = 1;
  for ( const auto &[id, class_name] : entries ) {
    const Eigen::Vector2i swatch_center( origin.x() + margin + row_height / 2,
                                         origin.y() + ( row + 1 ) * row_height );
    qp.save();
    geotiff_->drawObjectOfInterest(
        qp, swatch_center, std::to_string( id ), getClassColor( class_name ), { 255, 255, 255 },
        Eigen::Vector2f( 1.0f, 1.0f ), hector_geotiff_plugin_interface::Shape::SHAPE_DIAMOND, true,
        false );
    qp.restore();

    geotiff_->setFont( 6 );
    qp.setFont( geotiff_->getMapDrawFont() );
    qp.setPen( QColor( 0, 0, 0 ) );
    qp.drawText( origin.x() + 2 * margin + row_height,
                 origin.y() + ( row + 1 ) * row_height + row_height / 4,
                 QString::fromStdString( class_name ) );
    ++row;
  }
  qp.restore();
}

std::string
WorldmodelPlugin::autonomyModeToString( const autonomy_manager_msgs::msg::AutonomyMode &mode )
{
  // How it should be done
  // switch ( mode.autonomy_mode ) {
  // case 0:
  //   return "UNKNOWN";
  // case 1:
  //   return "TELEOPERATED";
  // case 2:
  //   return "SEMI_AUTONOMOUS";
  // case 3:
  //   return "AUTONOMOUS";
  // case 4:
  //   return "INACTIVE";
  // case 5:
  //   return "EMERGENCY_STOP_HARD";
  // case 6:
  //   return "EMERGENCY_STOP_SOFT";
  // default:
  //   return "UNKNOWN";
  // }
  // How we do it for RoboCup
  switch ( mode.autonomy_mode ) {
  case 3:
    return "A";
  case 1:
    return "T";
  default:
    return "A";
  }
}

void WorldmodelPlugin::writeToTextfile()
{
  auto exporter = geotiff_->getExporter();
  std::string path;
  if ( !exporter || !exporter->getExportPath( geotiff_->isAutosave(), path ) ) {
    RCLCPP_WARN( node_->get_logger(),
                 "Could not get export path for geotiff, cannot save maze objects to text file" );
    return;
  }

  const std::string filename = exporter->getExportName( true ) + "-pois.csv";
  const std::string total_path = path + "/" + filename;

  // Open the file in output mode with truncation
  std::ofstream file( total_path, std::ios::out | std::ios::trunc );
  if ( !file ) {
    RCLCPP_WARN( node_->get_logger(), "Could not open text file %s to save maze objects",
                 filename.c_str() );
    return;
  }

  auto info = exporter->getExportInfo();
  // Write to the file
  file << "pois\n";
  file << "1.3\n";
  file << info.team << "\n";
  file << info.country << "\n";
  file << hector_geotiff_plugin_interface::GeotiffExporterInterface::getDate( info.start_time, "-" )
       << "\n";
  file << hector_geotiff_plugin_interface::GeotiffExporterInterface::getTime( info.start_time, ":" )
       << "\n";
  file << info.mission << " " << info.mission_counter_ << "\n";

  int idx = 0;
  for ( const auto &[class_name, point, detection_time, operation_mode] : latest_object_list_ ) {
    file << idx++ << ", ";

    rclcpp::Time stamp( point.header.stamp );
    auto time_t_stamp = static_cast<time_t>( stamp.seconds() );
    file << std::put_time( std::gmtime( &time_t_stamp ), "%H:%M:%S" ) << ", ";

    const auto pos = point.point;
    file << pos.x << ", ";
    file << pos.y << ", ";
    file << pos.z << ", ";
    file << node_->get_namespace() << ", ";
    file << autonomyModeToString( operation_mode ) << "\n";
  }
}
} // namespace hector_worldmodel_geotiff_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS( hector_worldmodel_geotiff_plugin::WorldmodelPlugin,
                        hector_geotiff_plugin_interface::GeotiffPluginInterface )
