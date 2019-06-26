#include "pepper_base_manager/ArtificialLife.h"
#include "resource_management/message_storage/MessageWrapper.h"
#include <geometry_msgs/Twist.h>

namespace pepper_base_manager {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer)
{
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedTwist_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedTwist_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedTwist_data);
  geometry_msgs::Twist data;
  auto wrapped_PrioritizedTwist_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(data);
  wrapped_PrioritizedTwist_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_PrioritizedTwist_data);


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
  // auto wrapped_PrioritizedTwist_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedTwist_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedTwist_data);
  geometry_msgs::Twist data;
  auto wrapped_PrioritizedTwist_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(data);
  wrapped_PrioritizedTwist_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_PrioritizedTwist_data);
}

void ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedTwist_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedTwist_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedTwist_data);
}

} // namespace pepper_base_manager
