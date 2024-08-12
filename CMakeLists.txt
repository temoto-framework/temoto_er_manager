cmake_minimum_required(VERSION 3.8)
project(temoto_process_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(temoto_resource_registrar REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization program_options)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadProcess.srv"
  DEPENDENCIES rr_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${rr_interfaces_INCLUDE_DIRS}
    ${temoto_resource_registrar_INCLUDE_DIRS}
    ${temoto_resource_registrar_INCLUDE_DIRS}/rr
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_process_manager/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_resource_registrar/include
)

#
# Process Manager node
#
add_executable(process_manager_node
  src/process_manager.cpp
  src/process_manager_node.cpp
)

target_include_directories(process_manager_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${temoto_resource_registrar_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_process_manager/include
)

ament_target_dependencies(process_manager_node
  rclcpp
  std_msgs
  rr_interfaces
  temoto_resource_registrar
)

target_link_libraries(process_manager_node
  rr_core 
  Boost::filesystem
  Boost::serialization
  Boost::program_options
)

rosidl_target_interfaces(process_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  
install(TARGETS process_manager_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME})


# Example
add_executable(pm_client_node examples/test_pm_client_node.cpp)
target_include_directories(pm_client_node PUBLIC
  ${headers})
ament_target_dependencies(pm_client_node
  rclcpp
  std_msgs
  temoto_resource_registrar
)
target_link_libraries(pm_client_node
  rr_core
  Boost::filesystem
  Boost::serialization
  Boost::program_options
)
rosidl_target_interfaces(pm_client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS pm_client_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME})

  
# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install .srv files
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME})

    
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

ament_package()
