#ifndef OBJECT_TRACKER_OBJECT_H
#define OBJECT_TRACKER_OBJECT_H

#include <boost/shared_ptr.hpp>
#include <worldmodel_msgs/Object.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <ros/ros.h>

#include <map>

namespace hector_object_tracker {

class Object;
typedef boost::shared_ptr<Object> ObjectPtr;
typedef boost::shared_ptr<Object const> ObjectConstPtr;

class Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr<Object> Ptr;
    typedef boost::shared_ptr<Object const> ConstPtr;
    typedef worldmodel_msgs::ObjectState::_state_type State;

    Object(const std::string class_id = "", const std::string object_id = "");
    Object(const worldmodel_msgs::Object& other);
    virtual ~Object();

    static void reset();

    const worldmodel_msgs::Object& getObjectMessage() const {
      return object;
    }

    void getVisualization(visualization_msgs::MarkerArray &markers) const;

    const geometry_msgs::PoseWithCovariance& getPoseWithCovariance() const { return object.pose; }
    void setPose(const geometry_msgs::PoseWithCovariance& pose);

    const geometry_msgs::Pose& getPose() const { return object.pose.pose; }
    void setPose(const geometry_msgs::Pose& pose);
    void setPose(const tf::Pose& pose);

    void setPosition(const geometry_msgs::Point& position);
    void setPosition(const tf::Point& point);

    void setOrientation(const geometry_msgs::Quaternion& orientation);
    void setOrientation(const tf::Quaternion& orientation);

    const Eigen::Vector3f& getPosition() const;
    void setPosition(const Eigen::Vector3f& position);

    const Eigen::Matrix3f& getCovariance() const;
    void setCovariance(const Eigen::Matrix3f& covariance);
    void setCovariance(const tf::Matrix3x3& covariance);

    const std::string& getClassId() const {
      return object.info.class_id;
    }

    const std::string& getObjectId() const {
      return object.info.object_id;
    }

    void setObjectId(const std::string& object_id) {
      object.info.object_id = object_id;
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

    const std::string& getName() const {
      return object.info.name;
    }

    void setName(const std::string& name) {
      object.info.name = name;
    }

    std_msgs::Header getHeader() const {
      return object.header;
    }

    void setHeader(const std_msgs::Header &header) {
      object.header = header;
    }

    ros::Time getStamp() const {
      return object.header.stamp;
    }

    void intersect(const Eigen::Vector3f& position, const Eigen::Matrix3f& covariance, float support);
    void update(const Eigen::Vector3f& position, const Eigen::Matrix3f& covariance, float support);

    static void setNamespace(const std::string& ns);

    Object& operator=(const worldmodel_msgs::Object& other);

    ObjectPtr transform(tf::Transformer& tf, const std::string& target_frame) const;
    ObjectPtr transform(tf::Transformer& tf, const std::string& target_frame, const ros::Time& target_time) const;

private:
    ros::NodeHandle nh;
    worldmodel_msgs::Object object;

    Eigen::Vector3f position;
    Eigen::Matrix3f covariance;

    static std::map<std::string,unsigned int> object_count;
    static std::string object_namespace;
};

typedef Object::State ObjectState;

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_OBJECT_H
