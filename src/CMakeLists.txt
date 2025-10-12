add_library(trakstar_driver PointATC3DG.cpp)

target_link_libraries(trakstar_driver
  usb
)

install(TARGETS trakstar_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
