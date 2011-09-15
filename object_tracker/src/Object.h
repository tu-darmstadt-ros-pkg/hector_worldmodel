#ifndef OBJECT_TRACKER_OBJECT_H
#define OBJECT_TRACKER_OBJECT_H

#include <boost/shared_ptr.hpp>
#include <worldmodel_msgs/Object.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <map>

namespace object_tracker {

class Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr<Object> Ptr;
    typedef boost::shared_ptr<Object const> ConstPtr;
    typedef worldmodel_msgs::ObjectState::_state_type State;

    Object(const std::string class_id = "", const std::string object_id = "");
    virtual ~Object();

    static void reset();

    const worldmodel_msgs::Object& getObjectMessage() const {
      return object;
    }

    visualization_msgs::Marker getVisualization() const;

    const geometry_msgs::PoseWithCovariance& getPoseWithCovariance() const { return object.pose; }
    void setPose(const geometry_msgs::PoseWithCovariance& pose) { object.pose = pose; }

    const geometry_msgs::Pose& getPose() const { return object.pose.pose; }
    void setPose(const geometry_msgs::Pose& pose) { object.pose.pose = pose; }

    void setPosition(const geometry_msgs::Point& position) { object.pose.pose.position = position; }
    void setOrientation(const geometry_msgs::Quaternion& orientation) { object.pose.pose.orientation = orientation; }

    const Eigen::Vector3f& getPosition() const;
    void setPosition(const Eigen::Vector3f& position);

    const Eigen::Matrix3f& getCovariance() const;
    void setCovariance(const Eigen::Matrix3f& covariance);

    const std::string& getClassId() const {
      return object.info.class_id;
    }

    const std::string& getObjectId() const {
      return object.info.object_id;
    }

    float getSupport() const {
      return object.info.support;
    }

    void setSupport(float support) {
      object.info.support = support;
    }

    void addSupport(float support) {
      object.info.support += support;
    }

    State getState() const {
      return object.state.state;
    }

    void setState(const State& state) {
      object.state.state = state;
    }

    std_msgs::Header getHeader() const {
      return object.header;
    }

    void setHeader(const std_msgs::Header &header) {
      object.header = header;
    }

    void update(const Eigen::Vector3f& position, const Eigen::Matrix3f& covariance, float support);

    static void setNamespace(const std::string& ns);

private:
    ros::NodeHandle nh;
    worldmodel_msgs::Object object;

    Eigen::Vector3f position;
    Eigen::Matrix3f covariance;

    static std::map<std::string,unsigned int> object_count;
    static std::string object_namespace;
};

typedef Object::Ptr ObjectPtr;
typedef Object::ConstPtr ObjectConstPtr;
typedef Object::State ObjectState;

} // namespace object_tracker

#endif // OBJECT_TRACKER_OBJECT_H
