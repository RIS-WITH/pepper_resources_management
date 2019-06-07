#include "pepper_head_manager/pepper_head_manager.h"

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> PepperHeadManager::stateFromMsg(const pepper_head_manager_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.state_machine.states_PrioritizedJointTrajectory){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedPoint){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
PepperHeadManager::transitionFromMsg(const pepper_head_manager_msgs::StateMachine &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;

    for(auto x : msg.states_PrioritizedJointTrajectory){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }

    for(auto x : msg.states_PrioritizedPoint){
        for(auto t : x.header.transitions){
            transitions.push_back(
                        std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                            std::string(x.header.id),
                            std::string(t.next_state),
                            resource_management_msgs::EndCondition(t.end_condition)));
        }
    }
    return transitions;
}

pepper_head_manager_msgs::StateMachineRegister::Response PepperHeadManager::generateResponseMsg(uint32_t id)
{
  pepper_head_manager_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

void PepperHeadManager::publishPrioritizedJointTrajectoryMsg(naoqi_bridge_msgs::JointAngleTrajectory msg, bool is_new)
{
  // Put you own publishing function here
}

void PepperHeadManager::publishPrioritizedPointMsg(geometry_msgs::PointStamped msg, bool is_new)
{
  // Put you own publishing function here
}

