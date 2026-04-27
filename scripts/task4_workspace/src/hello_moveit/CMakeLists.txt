cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 添加可执行文件
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(hello_moveit
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
  moveit_ros_planning
  moveit_ros_planning_interface
)

# 安装
install(TARGETS
  hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()