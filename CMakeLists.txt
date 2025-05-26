
cmake_minimum_required(VERSION 3.5)
project(LDS_ROS_Driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)


include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(LDS_ROS_driver	src/LDS_ROS_driver.cpp)
target_link_libraries(LDS_ROS_driver ${Boost_LIBRARIES})
ament_target_dependencies(LDS_ROS_driver
  rclcpp
  std_msgs
  sensor_msgs
)


install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS LDS_ROS_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
