add_executable(trakstar_print PointATC3DG_test.cpp)

ament_target_dependencies(trakstar_print
  rclcpp
)

target_link_libraries(trakstar_print 
  trakstar_driver
)

install(TARGETS trakstar_print
  DESTINATION lib/${PROJECT_NAME}
)
