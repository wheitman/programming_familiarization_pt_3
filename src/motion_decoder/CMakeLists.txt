cmake_minimum_required(VERSION 3.5)
project(motion_decoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED) # The greatest package in history
ament_auto_find_build_dependencies() # Find dependencies listed in package.xml
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#ind_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(image_converter src/image_converter.cpp)
target_include_directories(image_converter PRIVATE include)
ament_target_dependencies(image_converter rclcpp OpenCV tf2_ros apriltag_msgs std_msgs image_transport cv_bridge sensor_msgs)

add_executable(motion_decoder src/motion_decoder.cpp)
target_include_directories(motion_decoder PRIVATE include)
ament_target_dependencies(motion_decoder rclcpp OpenCV tf2_ros apriltag_msgs std_msgs image_transport cv_bridge sensor_msgs)

target_link_libraries( image_converter ${OpenCV_LIBS} )
target_link_libraries( motion_decoder ${OpenCV_LIBS} )

install(TARGETS
  image_converter
  motion_decoder
  DESTINATION lib/${PROJECT_NAME})

ament_package()