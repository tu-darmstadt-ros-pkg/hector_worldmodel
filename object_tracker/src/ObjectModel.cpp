#include "ObjectModel.h"
#include "Object.h"

namespace object_tracker {

ObjectModel::ObjectModel()
{}

ObjectModel::~ObjectModel()
{}

ObjectModel::ObjectList ObjectModel::getObjects() const
{
  boost::recursive_mutex::scoped_lock lock(objectsMutex);
  return objects;
}

ObjectModel::ObjectList ObjectModel::getObjects(const std::string& class_id) const
{
  boost::recursive_mutex::scoped_lock lock(objectsMutex);
  ObjectList class_list;

  for(const_iterator it = begin(); it != end(); ++it) {
    if ((*it)->getClassId() == class_id) class_list.push_back(*it);
  }

  return class_list;
}

ObjectPtr ObjectModel::getObject(const std::string& object_id) const {
  boost::recursive_mutex::scoped_lock lock(objectsMutex);

  for(const_iterator it = begin(); it != end(); ++it) {
    if ((*it)->getObjectId() == object_id) return *it;
  }

  return ObjectPtr();
}

worldmodel_msgs::ObjectModelPtr ObjectModel::getObjectModelMessage() const {
  boost::recursive_mutex::scoped_lock lock(objectsMutex);
  worldmodel_msgs::ObjectModelPtr model(new worldmodel_msgs::ObjectModel());

  model->objects.reserve(objects.size());
  for(ObjectList::const_iterator it = objects.begin(); it != objects.end(); ++it)
    model->objects.push_back((*it)->getObjectMessage());

  return model;
}

void ObjectModel::reset()
{
  boost::recursive_mutex::scoped_lock lock(objectsMutex);
  objects.clear();
  Object::reset();
}

ObjectPtr ObjectModel::add(const std::string& class_id, const std::string& object_id) {
  return add(ObjectPtr(new Object(class_id, object_id)));
}

ObjectPtr ObjectModel::add(ObjectPtr object) {
  objects.push_back(object);
  return object;
}

void ObjectModel::remove(ObjectPtr object) {
  for(ObjectList::iterator it = objects.begin(); it != objects.end(); ++it) {
    if (*it == object) {
      remove(it);
      return;
    }
  }
}

void ObjectModel::remove(iterator it) {
  objects.erase(it);
}

void ObjectModel::getVisualization(visualization_msgs::MarkerArray &markers) const {
  boost::recursive_mutex::scoped_lock lock(objectsMutex);

  markers.markers.clear();
  for(ObjectList::const_iterator it = objects.begin(); it != objects.end(); ++it) {
    (*it)->getVisualization(markers);
  }
}

void ObjectModel::clearVisualization(visualization_msgs::MarkerArray &markers) const {
  getVisualization(markers);
  for(visualization_msgs::MarkerArray::_markers_type::iterator it = markers.markers.begin(); it != markers.markers.end(); ++it) {
    it->action = visualization_msgs::Marker::DELETE;
  }
}

} // namespace object_tracker
