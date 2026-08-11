cmake_minimum_required(VERSION 3.8)
project(beckhoff_ads_alarm_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ads_vendor REQUIRED)

add_executable(alarm_bridge_node
  src/alarm_bridge_node.cpp
  src/main.cpp
)

target_include_directories(alarm_bridge_node PUBLIC
  include
)

ament_target_dependencies(alarm_bridge_node
  rclcpp
  diagnostic_updater
  diagnostic_msgs
)

# ads_vendor exports a modern CMake target (ads_vendor::ads_vendor), not the classic
# ament_export_libraries/include_directories pair, so link it explicitly rather than
# through ament_target_dependencies -- matches beckhoff_ads_hardware_interface, the only
# proven-working link pattern for this vendor package in this workspace.
target_link_libraries(alarm_bridge_node
  ads_vendor::ads_vendor
)

install(
  TARGETS alarm_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
