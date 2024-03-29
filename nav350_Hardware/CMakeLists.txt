cmake_minimum_required(VERSION 3.8)
project(nav350_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include/nav350_Hardware)

add_library(${PROJECT_NAME}_lib
  "src/drivers/nav350/sicknav350/SickNAV350.cc"
  "src/drivers/nav350/sicknav350/SickNAV350BufferMonitor.cc"
  "src/drivers/nav350/sicknav350/SickNAV350Message.cc"
)

target_link_libraries(${PROJECT_NAME}_lib)

set(EXECUTABLE_NAME "nav350_Hardware")
add_executable(${EXECUTABLE_NAME} src/nav350_Hardware.cpp)

target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME}
  rclcpp sensor_msgs nav_msgs tf2_ros tf2)  



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY src launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/drivers
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
