cmake_minimum_required(VERSION 3.8)
project(vision_grasp_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 创建可执行脚本
install(PROGRAMS
  src/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  models
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()