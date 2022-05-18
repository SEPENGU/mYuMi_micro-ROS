cmake_minimum_required(VERSION 3.8)
project(transmission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

option(TRACETOOLS_DISABLED "Explicitly disable support for tracing with LTTng" OFF)
if(NOT TRACETOOLS_DISABLED)
  # Set TRACING_ENABLED if we can find lttng-ust
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(LTTNG lttng-ust)
    if(LTTNG_FOUND)
      set(TRACING_ENABLED TRUE)
    endif()
  endif()
endif()

# Tests
if(BUILD_TESTING)
  # tracetools is exported by rclcpp
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(example_interfaces REQUIRED)

  add_executable(pong src/pong.cpp)
  ament_target_dependencies(pong rclcpp std_msgs sensor_msgs example_interfaces)
  
  add_executable(test_ping src/test_ping.cpp)
  ament_target_dependencies(test_ping rclcpp std_msgs sensor_msgs)

  add_executable(test_pong src/test_pong.cpp)
  ament_target_dependencies(test_pong rclcpp std_msgs sensor_msgs)

  install(TARGETS
    pong
    test_ping
    test_pong
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Only build tracing tests if LTTng is enabled and found
  if(TRACING_ENABLED)
    find_package(tracetools REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_utils test/test_utils.cpp)
    if(TARGET test_utils)
      ament_target_dependencies(test_utils
        tracetools
      )
    endif()

    find_package(ament_cmake_pytest REQUIRED)
    set(_transmission_pytest_tests
      test/test_node.py
      test/test_pub_sub.py
      test/test_publisher.py
      test/test_subscription.py
      test/test_timer.py
    )
    foreach(_test_path ${_transmission_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  endif()

endif()

ament_package()
