cmake_minimum_required(VERSION 3.8)
project(pytrak)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find pybind11
find_package(pybind11 REQUIRED)
set(pybind11_DIR /usr/lib/python3/dist-packages/pybind11/share/cmake/pybind11)

include_directories(
  include
  /usr/include/usb
  /usr/include/python3.10
)

# Create the trakstar library
add_library(trakstar_lib
  src/PointATC3DG.cpp
)

target_link_libraries(trakstar_lib usb)

# Create the test executable
add_executable(PointATC3DG_test
  src/PointATC3DG.cpp
  tools/PointATC3DG_test.cpp
)

target_link_libraries(PointATC3DG_test usb)

# Create Python module using pybind11
add_library(pytrak MODULE
  src/python_wrapper.cpp
  src/PointATC3DG.cpp
)

target_link_libraries(pytrak PRIVATE
  usb
  pybind11::module
)

# Set Python module properties
set_target_properties(pytrak PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  PREFIX ""
  SUFFIX ".so"
)

# Install targets
install(TARGETS
  trakstar_lib
  PointATC3DG_test
  DESTINATION lib
)

# Install Python module
install(TARGETS pytrak
  DESTINATION lib/python3/dist-packages
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)