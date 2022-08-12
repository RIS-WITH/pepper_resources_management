cmake_minimum_required(VERSION 2.8.3)
project(pepper_head_human_following)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pepper_head_manager_msgs
  resource_management
  roscpp
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES pepper_head_human_following
 CATKIN_DEPENDS pepper_head_manager_msgs resource_management roscpp tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pepper_head_human_following.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME pepper_head_following PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)