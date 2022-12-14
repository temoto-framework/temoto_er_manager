cmake_minimum_required(VERSION 2.8.3)
project(temoto_process_manager)
add_compile_options(-std=c++14)

option(TEMOTO_ENABLE_TRACING_ "Use tracer" OFF)
if(TEMOTO_ENABLE_TRACING_)
  add_compile_options(-Dtemoto_enable_tracing)
endif()

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  temoto_resource_registrar
)

add_service_files(
  FILES
  LoadProcess.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs temoto_resource_registrar
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_process_manager 
  src/process_manager.cpp
  src/process_manager_node.cpp
)

add_dependencies(temoto_process_manager 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(temoto_process_manager 
  ${catkin_LIBRARIES}
)

install(TARGETS temoto_process_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)