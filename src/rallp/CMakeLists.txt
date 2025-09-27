cmake_minimum_required(VERSION 3.8)
project(rallp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch config world urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
  launch/online_async_launch.py
  launch/blue_dot_control.py
  launch/blue_dot_control2.py
  launch/robot_state_publisher.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
