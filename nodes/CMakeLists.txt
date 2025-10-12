add_executable(trakstar_node trakstar_node.cpp)

ament_target_dependencies(trakstar_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(trakstar_node
  trakstar
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(trakstar_node "${cpp_typesupport_target}")

install(TARGETS trakstar_node
  DESTINATION lib/${PROJECT_NAME}
)
