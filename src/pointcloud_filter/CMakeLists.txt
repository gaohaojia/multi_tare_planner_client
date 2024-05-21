cmake_minimum_required(VERSION 3.8)
project(pointcloud_filter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Build the executableinclude_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_executable(pointcloud_filter
  src/pointcloud_filter.cpp
)
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})


# Link libraries
ament_target_dependencies(pointcloud_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

# Install the executable and other files
install(TARGETS pointcloud_filter
  DESTINATION lib/${PROJECT_NAME}
)

# Configure package
ament_package()