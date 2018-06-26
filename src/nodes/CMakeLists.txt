add_executable(lucrezio_explorer_node lucrezio_explorer_node.cpp)

target_link_libraries(lucrezio_explorer_node
  frontier_detector_library
  ${catkin_LIBRARIES}
)

add_executable(lucrezio_semantic_explorer_node lucrezio_semantic_explorer_node.cpp)

target_link_libraries(lucrezio_semantic_explorer_node
  semantic_explorer_library
  ${catkin_LIBRARIES}
)

