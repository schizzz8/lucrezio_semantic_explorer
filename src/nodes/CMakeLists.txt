add_executable(lucrezio_explorer_node lucrezio_explorer_node.cpp)
target_link_libraries(lucrezio_explorer_node
  lucrezio_explorer_library
)

#add_dependencies(lucrezio_explorer_node
#   lucrezio_explorer_generate_messages_cpp
#)
