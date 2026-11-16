cmake_minimum_required(VERSION 3.8)
project(beckhoff_ads_client CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ads_vendor REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ads_client.cpp
  src/ads_client_utilities.cpp
  src/outage_tracker.cpp
  src/plc_type.cpp
  src/sum_requests.cpp
  src/write_handoff.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Threads::Threads
  PUBLIC ads_vendor::ads_vendor
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
      test_ads_client_utilities
      test_latest_sample_buffer
      test_plc_type
      test_sum_requests
      test_write_handoff
      test_outage_tracker
      test_ads_client
      test_ads_client_with_fake_plc)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE src)
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ads_vendor Threads)
ament_package()
