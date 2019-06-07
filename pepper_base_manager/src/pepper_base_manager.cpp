#include "pepper_base_manager/pepper_base_manager.h"

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> PepperBaseManager::stateFromMsg(const pepper_base_manager_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.state_machine.states_PrioritizedTwist){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
PepperBaseManager::transitionFromMsg(const pepper_base_manager_msgs::StateMachine &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;

    for(auto x : msg.states_PrioritizedTwist){
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

pepper_base_manager_msgs::StateMachineRegister::Response PepperBaseManager::generateResponseMsg(uint32_t id)
{
  pepper_base_manager_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

void PepperBaseManager::publishPrioritizedTwistMsg(geometry_msgs::Twist msg, bool is_new)
{
  _cmd_vel_pub.publish(msg);
}

