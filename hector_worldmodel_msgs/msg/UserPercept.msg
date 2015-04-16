# hector_worldmodel_msgs/PosePercept
# This message represents an observation given as input of an user.

# The header should equal the header of the corresponding image.
Header header

# The estimated pose of the object with its covariance
geometry_msgs/Pose pose

# Additional information about the percept
PerceptInfo info

