cmake_minimum_required(VERSION 2.8.3)
project(pepper_arm_manager_msgs)

find_package(catkin REQUIRED
roscpp
message_generation
resource_management_msgs
naoqi_bridge_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
FILES
StateMachineStatePrioritizedJointTrajectory.msg PrioritizedJointTrajectory.msg
StateMachine.msg)

## Generate services in the 'srv' folder
add_service_files(
FILES
StateMachineRegister.srv StateMachineExtract.srv
)

generate_messages(DEPENDENCIES resource_management_msgs naoqi_bridge_msgs)

catkin_package(CATKIN_DEPENDS message_runtime naoqi_bridge_msgs)
