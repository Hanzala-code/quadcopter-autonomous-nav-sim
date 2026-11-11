cmake_minimum_required(VERSION 3.8)
project(quad_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/quad_teleop.py
  scripts/go_to_position.py
  scripts/gps_odom_converter.py
  scripts/odom_tf_broadcaster.py
  scripts/path_planner.py
  scripts/obstacle_detector.py
  scripts/obstacle_tracker.py
  scripts/dynamic_obstacle_demo.py
  scripts/goal_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
