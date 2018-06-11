add_library(frontier_detector_library SHARED
  frontier_detector.cpp frontier_detector.h
)

target_link_libraries(frontier_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
