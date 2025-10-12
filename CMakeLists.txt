cmake_minimum_required(VERSION 3.8)
project(trakstar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrakstarMsg.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(include)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(nodes)

# Install Python executables
install(PROGRAMS
  nodes/calibration_data_collection.py
  nodes/publish_calibrated_data.py
  nodes/trakstar_mounting_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install udev rules
option(INSTALL_TRAKSTAR_UDEV_RULES "Install udev rules for Trakstar" ON)
if(INSTALL_TRAKSTAR_UDEV_RULES)
  install(
    FILES config/99-trakstar.rules
    DESTINATION "/etc/udev/rules.d"
    COMPONENT "udev"
  )
else()
  message(STATUS "Udev rules for trakstar not being installed, install them with -DINSTALL_TRAKSTAR_UDEV_RULES=ON")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(trakstar_driver)

ament_package()
