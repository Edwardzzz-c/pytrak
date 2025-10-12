add_library(trakstar PointATC3DG.cpp)

target_link_libraries(trakstar
  usb
)

install(TARGETS trakstar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
