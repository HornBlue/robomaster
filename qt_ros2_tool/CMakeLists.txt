cmake_minimum_required(VERSION 3.8...3.10)
project(qt_ros2_tool)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)

# 启用Qt的自动MOC、UIC和RCC - 这个很重要！
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 定义源文件和头文件
set(SOURCES
    src/main.cpp
    src/main_window.cpp
)

set(HEADERS
    include/main_window.hpp
)

# 如果有UI文件，取消下面的注释
# set(UI_FILES src/mainwindow.ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

# 添加可执行文件
add_executable(qt_ros2_tool
    ${SOURCES}
    ${HEADERS}  # 重要：包含头文件以便MOC处理
    # ${UI_FILES}
)

# 链接依赖
ament_target_dependencies(qt_ros2_tool
    rclcpp
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

add_executable(hit
    src/hit.cpp
)
ament_target_dependencies(hit
  rclcpp
  gazebo_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)





target_link_libraries(qt_ros2_tool
    Qt5::Core
    Qt5::Widgets
)

# 包含目录
target_include_directories(qt_ros2_tool PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(qt_ros2_tool PRIVATE
    ${Qt5Widgets_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)
target_link_libraries(qt_ros2_tool ${typesupport_target})

install(TARGETS
    qt_ros2_tool hit
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()