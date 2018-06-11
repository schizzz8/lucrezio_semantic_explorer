add_library(lucrezio_explorer_library SHARED
  lucrezio_explorer.cpp lucrezio_explorer.h
)

target_link_libraries(lucrezio_explorer_library
  frontier_detector_library
  ${catkin_LIBRARIES}
)
