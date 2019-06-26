#include "pepper_arm_manager/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"

#include <nao_interaction_msgs/GoToPosture.h>
#include <naoqi_bridge_msgs/JointAngleTrajectory.h>

namespace pepper_arm_manager {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer, const pepper_arm_manager::eSide side) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer), _side(side)
{
  ROS_INFO("test");
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  naoqi_bridge_msgs::JointAngleTrajectory data;
  if (_side == pepper_arm_manager::LEFT) {
    data.joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"};
    data.joint_angles = {1.55545652, 0.142660141, -1.20264077, -0.489339828, -0.174917936, 0.584358573};
  }else{
    data.joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};
    data.joint_angles = {1.56312633, -0.139592171, 1.204175, 0.492407799, 0.194776058, 0.583479762};
  }
  auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
}

void ArtificialLife::init()
{
  // Put our own initialisation function here
  // It will be called each time a new artificial life cycle begins

  // Set an initial value in the artificial life buffer
  // if you do not do that, at each new cycle, the resource
  // will start with the previous artificial life value

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  naoqi_bridge_msgs::JointAngleTrajectory data;
  if (_side == pepper_arm_manager::LEFT) {
    data.joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"};
    data.joint_angles = {1.55545652, 0.142660141, -1.20264077, -0.489339828, -0.174917936, 0.584358573};
  }else{
    data.joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};
    data.joint_angles = {1.56312633, -0.139592171, 1.204175, 0.492407799, 0.194776058, 0.583479762};
  }
  auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
}

void ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
}

} // namespace pepper_arm_manager
