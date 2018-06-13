add_executable(lucrezio_explorer_node lucrezio_explorer_node.cpp)

target_link_libraries(lucrezio_explorer_node
  frontier_detector_library
  ${catkin_LIBRARIES}
)

