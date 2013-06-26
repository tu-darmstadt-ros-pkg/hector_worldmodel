#include "Object.h"
#include <boost/lexical_cast.hpp>

#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#include "parameters.h"

namespace hector_object_tracker {

std::map<std::string,unsigned int> Object::object_count;
std::string Object::object_namespace;

Object::Object(const std::string class_id, const std::string object_id)
{
  if (!class_id.empty()) {
    object.info.class_id = class_id;
  } else {
    object.info.class_id = "object";
  }

  if (!object_id.empty()) {
    object.info.object_id = object_id;
  } else {
    object.info.object_id = object.info.class_id + "_" + boost::lexical_cast<std::string>(object_count[object.info.class_id]++);
  }

  // if (object.info.object_id[0] != '/') object.info.object_id = object_namespace + "/" + object.info.object_id;
}

Object::Object(const worldmodel_msgs::Object& other) {
  *this = other;
}

Object::~Object()
{}

void Object::reset() {
  object_count.clear();
}

void Object::setPose(const geometry_msgs::PoseWithCovariance& pose)
{
  setPose(pose.pose);
  object.pose.covariance = pose.covariance;
}

void Object::setPose(const geometry_msgs::Pose& pose)
{
  setPosition(pose.position);
  setOrientation(pose.orientation);
}

void Object::setPose(const tf::Pose &pose)
{
  setPosition(pose.getOrigin());
  tf::Quaternion rot;
  pose.getBasis().getRotation(rot);
  setOrientation(rot);
}

const Eigen::Vector3f& Object::getPosition() const {
  return position;
}

void Object::setPosition(const geometry_msgs::Point& position) {
  object.pose.pose.position = position;

  this->position.x() = position.x;
  this->position.y() = position.y;
  this->position.z() = position.z;
}

void Object::setPosition(const Eigen::Vector3f& position) {
  this->position = position;

  object.pose.pose.position.x = position.x();
  object.pose.pose.position.y = position.y();
  object.pose.pose.position.z = position.z();
}

void Object::setPosition(const tf::Point& position)
{
  this->position.x() = position.x();
  this->position.y() = position.y();
  this->position.z() = position.z();

  object.pose.pose.position.x = position.x();
  object.pose.pose.position.y = position.y();
  object.pose.pose.position.z = position.z();
}

void Object::setOrientation(const geometry_msgs::Quaternion& orientation)
{
  object.pose.pose.orientation = orientation;
}

void Object::setOrientation(const tf::Quaternion& orientation)
{
  object.pose.pose.orientation.w = orientation.w();
  object.pose.pose.orientation.x = orientation.x();
  object.pose.pose.orientation.y = orientation.y();
  object.pose.pose.orientation.z = orientation.z();
}

const Eigen::Matrix3f& Object::getCovariance() const {
  return covariance;
}

void Object::setCovariance(const Eigen::Matrix3f& eigen) {
  this->covariance = eigen;

  object.pose.covariance[0]  = covariance(0,0);
  object.pose.covariance[1]  = covariance(0,1);
  object.pose.covariance[2]  = covariance(0,2);
  object.pose.covariance[6]  = covariance(1,0);
  object.pose.covariance[7]  = covariance(1,1);
  object.pose.covariance[8]  = covariance(1,2);
  object.pose.covariance[12] = covariance(2,0);
  object.pose.covariance[13] = covariance(2,1);
  object.pose.covariance[14] = covariance(2,2);
}

void Object::setCovariance(const tf::Matrix3x3& tf) {
  this->covariance << tf[0][0], tf[0][1], tf[0][2],
                      tf[1][0], tf[1][1], tf[1][2],
                      tf[2][0], tf[2][1], tf[2][2];

  object.pose.covariance[0]  = covariance(0,0);
  object.pose.covariance[1]  = covariance(0,1);
  object.pose.covariance[2]  = covariance(0,2);
  object.pose.covariance[6]  = covariance(1,0);
  object.pose.covariance[7]  = covariance(1,1);
  object.pose.covariance[8]  = covariance(1,2);
  object.pose.covariance[12] = covariance(2,0);
  object.pose.covariance[13] = covariance(2,1);
  object.pose.covariance[14] = covariance(2,2);
}

void Object::intersect(const Eigen::Vector3f& positionB, const Eigen::Matrix3f& covarianceB, float support) {
  // old cov/covariance is A , new cov/covIn is B
  float omega = 0.5f;

  Eigen::Matrix3f A(covariance.inverse() * omega);
  Eigen::Matrix3f B(covarianceB.inverse() * (1.0f - omega));

  covariance = (A + B).inverse();
  position = covariance * (A * position + B * positionB);

  setPosition(position);
  setCovariance(covariance);
  addSupport(support);
}

void Object::update(const Eigen::Vector3f& positionB, const Eigen::Matrix3f& covarianceB, float support) {
  Eigen::Matrix3f A(covariance.inverse());
  Eigen::Matrix3f B(covarianceB.inverse());

  covariance = (A + B).inverse();
  position = covariance * (A * position + B * positionB);

  setPosition(position);
  setCovariance(covariance);
  addSupport(support);
}

void Object::getVisualization(visualization_msgs::MarkerArray &markers) const {
  visualization_msgs::Marker marker;
  std::string postfix;

  // default color
  marker.color = param(_marker_color, object.info.class_id);

  switch(object.state.state) {
    case worldmodel_msgs::ObjectState::CONFIRMED:
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
      postfix = " (CONFIRMED)";
      break;
    case worldmodel_msgs::ObjectState::DISCARDED:
      marker.color.a = 0.5;
      postfix = " (DISCARDED)";
      break;
    default:
      break;
  }

  marker.header = object.header;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = object.pose.pose;
  marker.ns = object.info.class_id;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  // markers.markers.push_back(marker);

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  markers.markers.push_back(marker);

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = (!object.info.name.empty() ? object.info.name : object.info.object_id) + postfix;
  marker.scale.x = 0.0;
  marker.scale.y = 0.0;
  marker.scale.z = 0.1;
  marker.pose.position.z += 1.5 * marker.scale.z;
  markers.markers.push_back(marker);
}

void Object::setNamespace(const std::string &ns) {
  object_namespace = ns;
}

Object& Object::operator =(const worldmodel_msgs::Object& other) {
  object = other;

  position.x() = object.pose.pose.position.x;
  position.y() = object.pose.pose.position.y;
  position.z() = object.pose.pose.position.z;

  covariance(0,0) = object.pose.covariance[0];
  covariance(0,1) = object.pose.covariance[1];
  covariance(0,2) = object.pose.covariance[2];
  covariance(1,0) = object.pose.covariance[6];
  covariance(1,1) = object.pose.covariance[7];
  covariance(1,2) = object.pose.covariance[8];
  covariance(2,0) = object.pose.covariance[12];
  covariance(2,1) = object.pose.covariance[13];
  covariance(2,2) = object.pose.covariance[14];

  return *this;
}

ObjectPtr Object::transform(tf::Transformer& tf, const std::string& target_frame) const
{
  return transform(tf, target_frame, object.header.stamp);
}

ObjectPtr Object::transform(tf::Transformer& tf, const std::string& target_frame, const ros::Time& target_time) const
{
  tf::StampedTransform transform;
  tf.lookupTransform(target_frame, object.header.frame_id, target_time, transform);

  ObjectPtr result(new Object(*this));

  tf::Pose pose;
  tf::poseMsgToTF(object.pose.pose, pose);

  // transform pose
  pose = transform * pose;
  result->setPose(pose);

  // rotate covariance matrix
  tf::Matrix3x3 rotation(transform.getBasis());
  tf::Matrix3x3 cov(covariance(0,0), covariance(0,1), covariance(0,2),
                    covariance(1,0), covariance(1,1), covariance(1,2),
                    covariance(2,0), covariance(2,1), covariance(2,2));
  result->setCovariance(rotation * cov * rotation.transpose());

  // set new frame_id
  result->object.header.frame_id = target_frame;

  return result;
}

} // namespace hector_object_tracker
