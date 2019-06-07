#include "pepper_head_manager/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"

namespace pepper_head_manager {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer)
{
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  // auto wrapped_PrioritizedPoint_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPoint_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_PrioritizedPoint_data);
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
  // auto wrapped_PrioritizedPoint_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPoint_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_PrioritizedPoint_data);
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
  // auto wrapped_PrioritizedPoint_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_PrioritizedPoint_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_PrioritizedPoint_data);
}

} // namespace pepper_head_manager
