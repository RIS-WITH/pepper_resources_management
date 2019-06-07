cmake_minimum_required(VERSION 2.8.3)
project(pepper_resources_synchronizer)

add_compile_options(-std=c++14 -Wall -Wextra -pthread)

find_package(catkin REQUIRED
roscpp
resource_management
resource_synchronizer
resource_management_msgs
resource_synchronizer_msgs
pepper_resources_synchronizer_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime pepper_resources_synchronizer_msgs)

find_package (Threads)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS}
)

add_executable(synchronizer src/pepper_resources_synchronizer.cpp)
add_dependencies(synchronizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(synchronizer
${catkin_LIBRARIES}
)

