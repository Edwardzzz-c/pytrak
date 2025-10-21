cmake_minimum_required(VERSION 3.8)
project(pytrak)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find pybind11
find_package(pybind11 REQUIRED)

# Find libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb)

include_directories(
  include
  ${LIBUSB_INCLUDE_DIRS}
  $ENV{CONDA_PREFIX}/include
)

link_directories($ENV{CONDA_PREFIX}/lib)

# Create the trakstar library
add_library(trakstar_lib
  src/PointATC3DG.cpp
)

target_link_libraries(trakstar_lib ${LIBUSB_LIBRARIES})

# Create the test executable
add_executable(PointATC3DG_test
  src/PointATC3DG.cpp
  tools/PointATC3DG_test.cpp
)

target_link_libraries(PointATC3DG_test ${LIBUSB_LIBRARIES})

# Create Python module using pybind11
add_library(pytrak MODULE
  src/python_wrapper.cpp
  src/PointATC3DG.cpp
)

target_link_libraries(pytrak PRIVATE
  ${LIBUSB_LIBRARIES}
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
