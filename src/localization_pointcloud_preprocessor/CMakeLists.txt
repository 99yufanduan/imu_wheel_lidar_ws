cmake_minimum_required(VERSION 3.14)
project(localization_pointcloud_preprocessor)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic )

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Suppress CGAL warning
set(CGAL_DATA_DIR ".")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
  SYSTEM
   ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pointcloud_preprocessor src/pointcloud_preprocessor.cpp src/passthrough_uint16.cpp )
target_include_directories(pointcloud_preprocessor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(pointcloud_preprocessor 
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_ros
  tf2_eigen
)

target_link_libraries(pointcloud_preprocessor
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)


install(TARGETS
  pointcloud_preprocessor  
  DESTINATION lib/${PROJECT_NAME})


ament_package()