cmake_minimum_required(VERSION 3.8)
project(ndt_matcher)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")


add_compile_options(-Wall -Wextra -Wpedantic) #-Wall：启用大多数的常规编译警告。-Wextra：启用一些额外的警告（在启用 -Wall 的基础上）。-Wpedantic：启用严格的 ISO C 和 ISO C++ 标准兼容性警告。
#common variables 
#CMAKE_CURRENT_SOURCE_DIR 指的是当前处理的 CMakeLists.txt 所在的路径,比如上面我们提到的 src 子目录。
#CMAKE_SOURCE_DIR 工程顶层目录
#PROJECT_NAME 返回通过 PROJECT 指令定义的项目名称。
#CMAKE_BUILD_TYPE 生成 debug 版和 release 版的程序。可以的取值是 Debug Release RelWithDebInfo 和 MinSizeRel。
#当这个变量值为 Debug 的时候,CMAKE_CXX_FLAGS_DEBUG 和 CMAKE_C_FLAGS_DEBUG 中的字符串作为编译选项生成 Makefile，当这个变量值为 Release 的时候，工程会使用变量CMAKE_CXX_FLAGS_RELEASE 和 CMAKE_C_FLAGS_RELEASE 选项生成 Makefile。
#CMAKE_CXX_FLAGS c++的编译属性 
#LIBRARY_OUTPUT_PATH 生成的库文件放置的路径
#使用ENV{NAME}指令就可以调用系统的环境变量了。比如：MESSAGE(STATUS “HOME dir: ENV{HOME}”)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ndt_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(ndt_matcher src/ndt_matcher.cpp src/pose_array_interpolator.cpp src/util_func.cpp)

target_link_libraries(ndt_matcher ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matcher 
geometry_msgs
nav_msgs  
sensor_msgs 
ndt_omp 
tf2 
tf2_ros 
rclcpp 
ndt_msgs 
pcl_ros 
tf2_eigen 
tf2_sensor_msgs
pcl_conversions
) #链接ros2库
add_executable(pcd_publisher_node src/pcd_publisher_node.cpp)
target_link_libraries(pcd_publisher_node ${PCL_LIBRARIES})
ament_target_dependencies(pcd_publisher_node rclcpp sensor_msgs pcl_conversions)

add_executable(pcdmap_transform src/pcdmap_transform.cpp)
target_link_libraries(pcdmap_transform ${PCL_LIBRARIES})

install(TARGETS
  ndt_matcher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pcd_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pcdmap_transform
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()