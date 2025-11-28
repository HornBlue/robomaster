cmake_minimum_required(VERSION 3.8)
project(teamX_challenge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

# 视觉节点
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node
  rclcpp sensor_msgs cv_bridge geometry_msgs image_transport std_msgs
)
target_link_libraries(vision_node ${OpenCV_LIBS})

# 决策节点 (新增)
add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node
  rclcpp geometry_msgs std_msgs
)

# 打击节点
add_executable(shooter_node src/shooter_node.cpp)
ament_target_dependencies(shooter_node rclcpp geometry_msgs)

# 安装所有节点
install(TARGETS
  vision_node decision_node shooter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch和config文件
install(DIRECTORY
  launch config results
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()