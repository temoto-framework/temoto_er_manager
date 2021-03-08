cmake_minimum_required(VERSION 2.8.3)
project(temoto_er_manager)
add_compile_options(-std=c++14)

option(TEMOTO_ENABLE_TRACING "Use tracer" OFF)

if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)
endif()

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  temoto_core
  temoto_resource_registrar
)

find_library(rr_core REQUIRED)

add_service_files(
  FILES
  LoadExtResource.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  temoto_core
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs temoto_core temoto_resource_registrar
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_er_manager 
  src/temoto_er_manager.cpp
  src/temoto_er_manager_node.cpp
)

add_dependencies(temoto_er_manager 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries( temoto_er_manager 
  ${catkin_LIBRARIES}
  rr_core
)

install(TARGETS temoto_er_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)