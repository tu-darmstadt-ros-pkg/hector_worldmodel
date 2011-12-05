#ifndef OBJECT_TRACKER_OBJECT_TRACKER_H
#define OBJECT_TRACKER_OBJECT_TRACKER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <worldmodel_msgs/ImagePercept.h>
#include <worldmodel_msgs/PosePercept.h>
#include <worldmodel_msgs/SetObjectState.h>
#include <worldmodel_msgs/AddObject.h>
#include <worldmodel_msgs/GetObjectModel.h>

#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <hector_marker_drawing/HectorDrawings.h>

#include <map>

#include "ObjectModel.h"

namespace object_tracker {

class ObjectTracker {
public:
  ObjectTracker();
  virtual ~ObjectTracker();

  void sysCommandCb(const std_msgs::StringConstPtr &);
  void imagePerceptCb(const worldmodel_msgs::ImagePerceptConstPtr &);
  void posePerceptCb(const worldmodel_msgs::PosePerceptConstPtr &);

  bool setObjectStateCb(worldmodel_msgs::SetObjectState::Request& request, worldmodel_msgs::SetObjectState::Response& response);
  bool addObjectCb(worldmodel_msgs::AddObject::Request& request, worldmodel_msgs::AddObject::Response& response);
  bool getObjectModelCb(worldmodel_msgs::GetObjectModel::Request& request, worldmodel_msgs::GetObjectModel::Response& response);

  ObjectModel& getModel()             { return model; }
  const ObjectModel& getModel() const { return model; }

  void publishModel();

protected:
  bool transformPose(const geometry_msgs::Pose& from, geometry_msgs::Pose &to, std_msgs::Header &header, tf::StampedTransform *transform = 0);
  bool transformPose(const geometry_msgs::PoseWithCovariance& from, geometry_msgs::PoseWithCovariance &to, std_msgs::Header &header);
  bool mapToNextObstacle(const geometry_msgs::Pose& source, const std_msgs::Header &header, geometry_msgs::Pose &mapped);

private:
  ros::NodeHandle nh;
  ros::Subscriber imagePerceptSubscriber;
  ros::Subscriber posePerceptSubscriber;
  ros::Subscriber sysCommandSubscriber;

  ros::Publisher modelPublisher;
  ros::Publisher modelUpdatePublisher;
  ros::Publisher poseDebugPublisher;
  ros::Publisher pointDebugPublisher;

  ros::ServiceClient distanceToObstacle;
  ros::ServiceServer setObjectState;
  ros::ServiceServer addObject;
  ros::ServiceServer getObjectModel;
  std::vector<ros::ServiceClient> verificationServices;

  tf::TransformListener tf;

  HectorDrawings drawings;

  ObjectModel model;
  typedef boost::shared_ptr<image_geometry::PinholeCameraModel> CameraModelPtr;
  std::map<std::string,CameraModelPtr> cameraModels;

  bool _project_objects;
  std::string _frame_id;
  std::string _worldmodel_ns;
  double _default_distance;
  double _distance_variance;
  double _angle_variance;
  double _min_height;
  double _max_height;
};

} // namespace object_tracker

#endif // OBJECT_TRACKER_OBJECT_TRACKER_H
