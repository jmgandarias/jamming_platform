cmake_minimum_required(VERSION 3.8)
project(jamming_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_ros2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(force_displacement_experiment src/force_displacement_experiment.cpp)
target_include_directories(force_displacement_experiment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(force_displacement_experiment rclcpp std_msgs sensor_msgs geometry_msgs dynamixel_sdk dynamixel_ros2)

if(DEFINED dynamixel_ros2_LIBRARIES AND dynamixel_ros2_LIBRARIES)
  target_link_libraries(force_displacement_experiment ${dynamixel_ros2_LIBRARIES})
elseif(TARGET dynamixel_ros2_lib)
  target_link_libraries(force_displacement_experiment dynamixel_ros2_lib)
else()
  message(FATAL_ERROR "dynamixel_ros2 did not export a usable library for linking")
endif()

install(TARGETS force_displacement_experiment
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
