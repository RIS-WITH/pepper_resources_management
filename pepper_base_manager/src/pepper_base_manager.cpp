#include "pepper_base_manager/pepper_base_manager.h"
#include <tf2/LinearMath/Quaternion.h>

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> PepperBaseManager::stateFromMsg(const pepper_base_manager_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.state_machine.states_PrioritizedTwist){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<geometry_msgs::Twist>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedAngle){
      auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<float>>(x.data);
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
    for(auto x : msg.states_PrioritizedAngle){
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
  if (is_new) {
    _cmd_vel_pub.publish(msg);
  }
}

void PepperBaseManager::publishPrioritizedAngleMsg(float msg, bool is_new) {
  if (is_new){
    nao_interaction_msgs::GoToPose r;
    r.request.pose.header.frame_id = "/base_link";
    r.request.pose.header.stamp = ros::Time::now();
    tf2::Quaternion q({0.0, 0.0, 1.0}, msg);
    r.request.pose.pose.orientation.x = q.x();
    r.request.pose.pose.orientation.y = q.y();
    r.request.pose.pose.orientation.z = q.z();
    r.request.pose.pose.orientation.w = q.w();
    _moveToSrv.call(r);
    done();
  }
}

