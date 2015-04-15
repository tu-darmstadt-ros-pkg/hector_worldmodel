# hector_worldmodel_msgs/ObjectInfo
# This message contains information about the estimated class affiliation, object id and corresponding support

# A string identifying the object's class (all objects of a class look the same)
string class_id

# A string identifying the specific object
string object_id

# A string that contains the name or a description of the specific object
string name

# The support (degree of belief) of the object's presence given as log odd ratio
float32 support

# A float array that can contain additional data for the percept (e.g manometer values)
float32[] data

# A float array that can contain fixed properties (e.g as size,diameter)
float32[] fixed_properties
