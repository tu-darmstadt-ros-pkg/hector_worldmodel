#ifndef OBJECT_TRACKER_OBJECTMODEL_H
#define OBJECT_TRACKER_OBJECTMODEL_H

#include <boost/shared_ptr.hpp>
#include <worldmodel_msgs/ObjectModel.h>
#include <worldmodel_msgs/ImagePercept.h>
#include <worldmodel_msgs/PosePercept.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <string>
#include <boost/thread/recursive_mutex.hpp>

namespace object_tracker {

class Object;

class ObjectModel
{
public:
    typedef boost::shared_ptr<Object> ObjectPtr;
    typedef boost::shared_ptr<Object const> ObjectConstPtr;

    typedef std::list<ObjectPtr> ObjectList;
    typedef ObjectList::iterator iterator;
    typedef ObjectList::const_iterator const_iterator;

public:
    ObjectModel();
    virtual ~ObjectModel();

    ObjectList getObjects() const;
    ObjectList getObjects(const std::string& class_id) const;
    ObjectPtr getObject(const std::string& object_id) const;

    worldmodel_msgs::ObjectModelPtr getObjectModelMessage() const;
    void getVisualization(visualization_msgs::MarkerArray &markers) const;
    void clearVisualization(visualization_msgs::MarkerArray &markers) const;
    void reset();

    iterator begin() { return objects.begin(); }
    iterator end()   { return objects.end(); }
    const_iterator begin() const { return objects.begin(); }
    const_iterator end() const { return objects.end(); }

    ObjectPtr create(const std::string& class_id = "", const std::string& object_id = "");
    ObjectPtr add(const std::string& class_id = "", const std::string& object_id = "");
    ObjectPtr add(ObjectPtr object);
    void remove(ObjectPtr object);
    void remove(iterator it);

    void lock() const { objectsMutex.lock(); }
    bool try_lock() const { return objectsMutex.try_lock(); }
    void unlock() const { objectsMutex.unlock(); }

private:
    ObjectModel(const ObjectModel&);
    ObjectList objects;
    mutable boost::recursive_mutex objectsMutex;
};

} // namespace object_tracker

#endif // OBJECT_TRACKER_OBJECTMODEL_H
