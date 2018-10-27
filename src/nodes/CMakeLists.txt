add_executable(explorer_node explorer_node.cpp)

target_link_libraries(explorer_node
  frontier_detector_library
  ${catkin_LIBRARIES}
)

add_executable(semantic_explorer_node semantic_explorer_node.cpp)

target_link_libraries(semantic_explorer_node
  semantic_explorer_library
  ${catkin_LIBRARIES}
)

