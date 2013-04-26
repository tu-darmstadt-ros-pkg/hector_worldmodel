set(SOURCES
  object_tracker.cpp
  object_tracker.h
  Object.cpp
  Object.h
  ObjectModel.cpp
  ObjectModel.h
  parameters.h
  parameters.cpp
)

rosbuild_add_executable(object_tracker ${SOURCES})
