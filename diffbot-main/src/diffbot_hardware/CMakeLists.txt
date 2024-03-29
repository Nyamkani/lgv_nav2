cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diffbot_msg REQUIRED)



## COMPILE
# target_include_directories(
#   ${PROJECT_NAME}
#   PUBLIC
#   include
# )

include_directories(
  include
)

add_library(
  ${PROJECT_NAME} SHARED
  src/diffbot_system.cpp
  #src/diffbot_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp  
  pluginlib  
  hardware_interface
  rclcpp_lifecycle
  diffbot_msg
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
#target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
#target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
# endif()





# EXPORTS
# ament_export_include_directories(
#  include
# )
# ament_export_libraries(
#  ${PROJECT_NAME}
# )
# ament_export_dependencies(e
#  hardware_interface
#  pluginlib
#  rclcpp
#  rclcpp_lifecycle
# )




#ament_export_targets(export_my_library HAS_LIBRARY_TARGET)
#ament_export_dependencies(some_dependency)

# install(
#    DIRECTORY include/
#    DESTINATION include
# )

# install(
#   #TARGETS my_library 
#   EXPORT export_my_library
#   LIBRARY DESTINATION lib 
#   ARCHIVE DESTINATION lib 
#   RUNTIME DESTINATION bin 
#   INCLUDES DESTINATION include
# )




pluginlib_export_plugin_description_file(hardware_interface diffbot_hardware.xml)



ament_package()