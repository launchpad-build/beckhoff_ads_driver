cmake_minimum_required(VERSION 3.8)
project(beckhoff_ads_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(beckhoff_ads_client REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  beckhoff_ads_hardware_interface
  SHARED
  src/beckhoff_ads_hardware_interface.cpp
)
target_compile_features(beckhoff_ads_hardware_interface PUBLIC cxx_std_20)
target_include_directories(
  beckhoff_ads_hardware_interface
  PUBLIC
  include
)
target_link_libraries(beckhoff_ads_hardware_interface PUBLIC
  beckhoff_ads_client::beckhoff_ads_client
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

target_compile_definitions(beckhoff_ads_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface beckhoff_ads_hardware_interface.xml)

install(
  TARGETS
  beckhoff_ads_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  beckhoff_ads_hardware_interface
)
ament_export_dependencies(
  beckhoff_ads_client
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interface_initial_value
    test/test_interface_initial_value.cpp
  )
  target_link_libraries(test_interface_initial_value beckhoff_ads_hardware_interface)

  ament_add_gtest(test_hardware_interface_with_fake_plc
    test/test_hardware_interface_with_fake_plc.cpp
  )
  target_link_libraries(test_hardware_interface_with_fake_plc beckhoff_ads_hardware_interface)
endif()

ament_package()
