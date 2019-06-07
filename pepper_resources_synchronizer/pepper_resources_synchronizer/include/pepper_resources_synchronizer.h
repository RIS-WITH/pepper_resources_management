#ifndef pepper_resources_synchronizer_RESOURCE_SYNCHRONIZER_H
#define pepper_resources_synchronizer_RESOURCE_SYNCHRONIZER_H

#include <map>
#include <mutex>

#include <ros/ros.h>

#include "resource_management_msgs/StateMachinesCancel.h"
#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"

#include "pepper_resources_synchronizer_msgs/MetaStateMachineRegister.h"
#include "resource_synchronizer/StateMachinesHolder.h"
#include "resource_synchronizer/StateMachinesManager.h"
#include "resource_synchronizer/synchronizer/StateMachinesSynchronizer.h"

#include "pepper_arm_manager_msgs/StateMachineRegister.h"
#include "pepper_arm_manager_msgs/StateMachineExtract.h"
#include "pepper_base_manager_msgs/StateMachineRegister.h"
#include "pepper_base_manager_msgs/StateMachineExtract.h"
#include "pepper_head_manager_msgs/StateMachineRegister.h"
#include "pepper_head_manager_msgs/StateMachineExtract.h"

namespace pepper_resources_synchronizer
{

class PepperResourcesSynchronizer
{
public:
  PepperResourcesSynchronizer(ros::NodeHandlePtr nh);

  void publishStatus(resource_synchronizer::SubStateMachineStatus status);

  void run();

private:
  ros::NodeHandlePtr _nh;
  resource_synchronizer::StateMachinesHolder<pepper_resources_synchronizer_msgs::SubStateMachine_pepper_head_manager_msgs, pepper_head_manager_msgs::StateMachineRegister, pepper_head_manager_msgs::StateMachineExtract> _holder_pepper_head_manager;
  resource_synchronizer::StateMachinesHolder<pepper_resources_synchronizer_msgs::SubStateMachine_pepper_arm_manager_msgs, pepper_arm_manager_msgs::StateMachineRegister, pepper_arm_manager_msgs::StateMachineExtract> _holder_pepper_arm_manager_left;
  resource_synchronizer::StateMachinesHolder<pepper_resources_synchronizer_msgs::SubStateMachine_pepper_arm_manager_msgs, pepper_arm_manager_msgs::StateMachineRegister, pepper_arm_manager_msgs::StateMachineExtract> _holder_pepper_arm_manager_right;
  resource_synchronizer::StateMachinesHolder<pepper_resources_synchronizer_msgs::SubStateMachine_pepper_base_manager_msgs, pepper_base_manager_msgs::StateMachineRegister, pepper_base_manager_msgs::StateMachineExtract> _holder_pepper_base_manager;
  resource_synchronizer::StateMachinesManager _manager;
  resource_synchronizer::StateMachinesSynchronizer _synchronizer;

  unsigned int _current_id;
  std::map<int, resource_synchronizer_msgs::MetaStateMachinesStatus> _status;
  std::mutex mutex_;

  ros::ServiceServer _register_service;
  ros::Publisher _state_machine_status_publisher;
  ros::ServiceServer _state_machine_cancel_service;

  bool registerMetaStateMachine(pepper_resources_synchronizer_msgs::MetaStateMachineRegister::Request &req,
                                pepper_resources_synchronizer_msgs::MetaStateMachineRegister::Response &res);

  bool stateMachineCancel
      (resource_management_msgs::StateMachinesCancel::Request  &req,
      resource_management_msgs::StateMachinesCancel::Response &res);

  void removeStatusIfNeeded(int id);

  std::vector<std::string> getSynchros(std::string event);
  std::vector<std::string> split(const std::string& str, const std::string& delim);
};

} // namespace pepper_resources_synchronizer

#endif // pepper_resources_synchronizer_RESOURCE_SYNCHRONIZER_H
