#include "pepper_resources_synchronizer.h"

#include <thread>

#include "resource_synchronizer_msgs/MetaStateMachineHeader.h"
#include "resource_synchronizer_msgs/MetaStateMachinesStatus.h"

namespace pepper_resources_synchronizer
{

PepperResourcesSynchronizer::PepperResourcesSynchronizer(ros::NodeHandlePtr nh) : _nh(std::move(nh)),
  _holder_pepper_head_manager("pepper_head_manager"),
  _holder_pepper_arm_manager_left("pepper_arm_manager_left"),
  _holder_pepper_arm_manager_right("pepper_arm_manager_right"),
  _holder_pepper_base_manager("pepper_base_manager"),
  _current_id(0)
{

  _register_service = _nh->advertiseService("state_machines_register", &PepperResourcesSynchronizer::registerMetaStateMachine, this);
  _state_machine_status_publisher = _nh->advertise<resource_synchronizer_msgs::MetaStateMachinesStatus>("state_machine_status", 100);
  _state_machine_cancel_service = _nh->advertiseService("state_machine_cancel", &PepperResourcesSynchronizer::stateMachineCancel, this);

  _holder_pepper_head_manager.registerSatusCallback([this](auto status){ this->publishStatus(status); });
  _holder_pepper_arm_manager_left.registerSatusCallback([this](auto status){ this->publishStatus(status); });
  _holder_pepper_arm_manager_right.registerSatusCallback([this](auto status){ this->publishStatus(status); });
  _holder_pepper_base_manager.registerSatusCallback([this](auto status){ this->publishStatus(status); });

  _manager.registerHolder(&_holder_pepper_head_manager);
  _manager.registerHolder(&_holder_pepper_arm_manager_left);
  _manager.registerHolder(&_holder_pepper_arm_manager_right);
  _manager.registerHolder(&_holder_pepper_base_manager);
  _manager.registerSatusCallback([this](auto status){ this->publishStatus(status); });

  ROS_INFO("pepper_resources_synchronizer ready.");
}

void PepperResourcesSynchronizer::publishStatus(resource_synchronizer::SubStateMachineStatus status)
{
  mutex_.lock();
  auto it = _status.find(status.id);
  if(it != _status.end())
  {
    int sub_id = -1;
    for(size_t i = 0; i < it->second.resource.size(); i++)
      if(it->second.resource[i] == status.resource)
      {
        sub_id = i;
        break;
      }

    if(sub_id != -1)
    {
      if(it->second.state_name[sub_id] != status.state_name)
        _synchronizer.reset(status.id);
      auto tmp = getSynchros(status.event_name);
      for(auto s : tmp)
        _synchronizer.activate(status.id, s, status.resource);

      it->second.state_name[sub_id] = status.state_name;
      it->second.state_event[sub_id] = status.event_name;
      _state_machine_status_publisher.publish(it->second);
      removeStatusIfNeeded(sub_id);
    }
  }
  mutex_.unlock();
}

void PepperResourcesSynchronizer::run()
{
  std::thread pepper_resources_synchronizer_thread(&resource_synchronizer::StateMachinesManager::run, &_manager);

  ros::spin();

  _manager.stop();
  pepper_resources_synchronizer_thread.join();
}

bool PepperResourcesSynchronizer::registerMetaStateMachine(pepper_resources_synchronizer_msgs::MetaStateMachineRegister::Request &req,
pepper_resources_synchronizer_msgs::MetaStateMachineRegister::Response &res){

  _manager.halt();

  _status[_current_id].id = _current_id;
  bool inserted;

  inserted = _holder_pepper_head_manager.insert(_current_id, req.state_machine_pepper_head_manager, req.header.priority);
  if(inserted)
  {
    _status[_current_id].resource.push_back("pepper_head_manager");
    _synchronizer.insert(_current_id, "pepper_head_manager", _holder_pepper_head_manager.getSynchros(_current_id));
  }

  inserted = _holder_pepper_arm_manager_left.insert(_current_id, req.state_machine_pepper_arm_manager_left, req.header.priority);
  if(inserted)
  {
    _status[_current_id].resource.push_back("pepper_arm_manager_left");
    _synchronizer.insert(_current_id, "pepper_arm_manager_left", _holder_pepper_arm_manager_left.getSynchros(_current_id));
  }

  inserted = _holder_pepper_arm_manager_right.insert(_current_id, req.state_machine_pepper_arm_manager_right, req.header.priority);
  if(inserted)
  {
    _status[_current_id].resource.push_back("pepper_arm_manager_right");
    _synchronizer.insert(_current_id, "pepper_arm_manager_right", _holder_pepper_arm_manager_right.getSynchros(_current_id));
  }

  inserted = _holder_pepper_base_manager.insert(_current_id, req.state_machine_pepper_base_manager, req.header.priority);
  if(inserted)
  {
    _status[_current_id].resource.push_back("pepper_base_manager");
    _synchronizer.insert(_current_id, "pepper_base_manager", _holder_pepper_base_manager.getSynchros(_current_id));
  }

  _status[_current_id].resource.push_back("_");
  _manager.insert(_current_id, req.header);

  res.id = _current_id;

  _status[_current_id].state_name.resize(_status[_current_id].resource.size(), "_");
  _status[_current_id].state_name[_status[_current_id].state_name.size() - 1] = ""; // global status as no state
  _status[_current_id].state_event.resize(_status[_current_id].resource.size());
  _current_id++;

  _manager.realease();

  return true;
}

bool PepperResourcesSynchronizer::stateMachineCancel
                    (resource_management_msgs::StateMachinesCancel::Request  &req,
                    resource_management_msgs::StateMachinesCancel::Response &res)
{
  bool done = true;
  done = done || _holder_pepper_head_manager.cancel(req.id);
  done = done || _holder_pepper_arm_manager_left.cancel(req.id);
  done = done || _holder_pepper_arm_manager_right.cancel(req.id);
  done = done || _holder_pepper_base_manager.cancel(req.id);

  res.ack = done;

  return true;
}

void PepperResourcesSynchronizer::removeStatusIfNeeded(int id)
{
  bool remove = true;
  for(size_t i = 0; i < _status[id].state_name.size(); i++)
    if(_status[id].state_name[i] != "")
      remove = false;

  if(remove)
    _status.erase(id);
}

std::vector<std::string> PepperResourcesSynchronizer::getSynchros(std::string event)
{
  std::vector<std::string> res;

  if(event.find("wait_synchro_") == 0)
  {
    event = event.substr(13);
    res = split(event, "_");
  }

  return res;
}

std::vector<std::string> PepperResourcesSynchronizer::split(const std::string& str, const std::string& delim)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
      pos = str.find(delim, prev);
      if (pos == std::string::npos)
        pos = str.length();

      std::string token = str.substr(prev, pos-prev);

      if (!token.empty())
        tokens.push_back(token);
      prev = pos + delim.length();
    }
    while ((pos < str.length()) && (prev < str.length()));

    return tokens;
}

} // namespace pepper_resources_synchronizer

int main(int argc, char** argv){
  ros::init(argc, argv, "pepper_resources_synchronizer");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  resource_synchronizer::StateMachineSynchroHolder::setNodeHandle(nh);

  pepper_resources_synchronizer::PepperResourcesSynchronizer syn(nh);

  syn.run();

  return 0;
}
