#ifndef OBJECT_MODEL_UTIL_H
#define OBJECT_MODEL_UTIL_H

namespace hector_worldmodel_msgs
{

hector_worldmodel_msgs::Object* getObjectByID(hector_worldmodel_msgs::ObjectModel &object_model, std::string object_id){

    // [object_id - 1] seems brittle
    for(std::vector<hector_worldmodel_msgs::Object>::iterator it = object_model.objects.begin(); it != object_model.objects.end(); ++it) {
        /* std::cout << *it; ... */
        std::string object_name = it->info.name;
        if (object_name.find(object_id) != std::string::npos) {
            ROS_INFO("Object selected for id %s: %s", object_id.c_str(), object_name.c_str());
            return &(*it);
        }
    }
    return NULL;
}

}

#endif
