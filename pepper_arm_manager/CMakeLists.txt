cmake_minimum_required(VERSION 2.8.3)
project(pepper_arm_manager)
add_compile_options(-std=gnu++14 -Wall -pthread)
find_package(catkin REQUIRED
    roscpp
    pepper_arm_manager_msgs
    resource_management
    nao_interaction_msgs
    pluginlib
    geometry_msgs
    tf2
    tf2_ros
)

catkin_package(
#INCLUDE_DIRS include
#LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp message_runtime
# DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp src/${PROJECT_NAME}.cpp src/ArtificialLife.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_synchronised src/main_synchronised.cpp src/${PROJECT_NAME}.cpp src/ArtificialLife.cpp)
add_dependencies(${PROJECT_NAME}_synchronised ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_synchronised ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(tests_${PROJECT_NAME} test/main.test test/test.cpp)
    add_dependencies(tests_${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(tests_${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
