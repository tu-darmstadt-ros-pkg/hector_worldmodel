set(SOURCES
  object_tracker.cpp
  object_tracker.h
  Object.cpp
  Object.h
  ObjectModel.cpp
  ObjectModel.h
)

rosbuild_add_executable(${PROJECT_NAME} ${SOURCES})
