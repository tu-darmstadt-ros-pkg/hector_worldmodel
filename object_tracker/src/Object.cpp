#include "Object.h"
#include <boost/lexical_cast.hpp>

namespace object_tracker {

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

  if (object.info.object_id[0] != '/') object.info.object_id = object_namespace + "/" + object.info.object_id;
}

Object::~Object()
{}

void Object::reset() {
  object_count.clear();
}

const Eigen::Vector3f& Object::getPosition() const {
  return position;
}

void Object::setPosition(const Eigen::Vector3f& position) {
  this->position = position;

  object.pose.pose.position.x = position.x();
  object.pose.pose.position.y = position.y();
  object.pose.pose.position.z = position.z();
  object.pose.pose.orientation.w = 1.0;
}

const Eigen::Matrix3f& Object::getCovariance() const {
  return covariance;
}

void Object::setCovariance(const Eigen::Matrix3f& covariance) {
  this->covariance = covariance;

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
  marker.header = object.header;

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = object.pose.pose;
  marker.ns = "worldmodel";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.b = 1.0;
  marker.color.a = std::max(0.0, std::min(1.0, object.info.support / 20.0));
  markers.markers.push_back(marker);

  marker.header = object.header;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = object.pose.pose;
  marker.ns = "worldmodel";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = !object.info.name.empty() ? object.info.name : object.info.object_id;
  marker.scale.x = 0.0;
  marker.scale.y = 0.0;
  marker.scale.z = 0.1;
  marker.pose.position.z += 1.5 * marker.scale.z;
  markers.markers.push_back(marker);
}

void Object::setNamespace(const std::string &ns) {
  object_namespace = ns;
}

} // namespace object_tracker
