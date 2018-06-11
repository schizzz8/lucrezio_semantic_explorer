add_executable(frontier_detector_app frontier_detector_app.cpp)

target_link_libraries(frontier_detector_app
  yaml-cpp
  frontier_detector_library
 )

