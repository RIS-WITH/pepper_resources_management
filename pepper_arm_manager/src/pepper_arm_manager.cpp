#include "pepper_arm_manager/pepper_arm_manager.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> PepperArmManager::stateFromMsg(const pepper_arm_manager_msgs::StateMachineRegister::Request &msg)
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
PepperArmManager::transitionFromMsg(const pepper_arm_manager_msgs::StateMachine &msg)
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
            std::make_tuple<std::string, std::string, resource_management_msgs::EndCondition>(
                std::string(x.header.id),
                std::string(t.next_state),
                resource_management_msgs::EndCondition(t.end_condition)
                )
            );
      }
    }
    return transitions;
}

pepper_arm_manager_msgs::StateMachineRegister::Response PepperArmManager::generateResponseMsg(uint32_t id)
{
  pepper_arm_manager_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

void PepperArmManager::publishPrioritizedJointTrajectoryMsg(naoqi_bridge_msgs::JointAngleTrajectory msg, bool is_new)
{
  if (is_new){
    nao_interaction_msgs::MotionSetAnglesRequest req;
    nao_interaction_msgs::MotionSetAngles r;
    for (size_t i=0; i < msg.joint_names.size(); i++){
      req.names.push_back(msg.joint_names[i]);
      req.angles.push_back(msg.joint_angles[i]);
    }
    float speed = std::max(std::min(ARM_MAX_SPEED, _nh->param(ARM_SPEED_PARAM, ARM_MIN_SPEED)), ARM_MIN_SPEED);
    req.max_speed_fraction = speed;
    r.request = req;
    _setAnglesSrv.call(r);
    done();
  }
}

void PepperArmManager::publishPrioritizedPointMsg(geometry_msgs::PointStamped msg, bool is_new){
  if (is_new) {  // TODO: Check if the point is the same in robot frame...
    geometry_msgs::PointStamped point;
    if (msg.header.frame_id !=
        "/base_link") {
      _tfBuffer.transform<geometry_msgs::PointStamped>(msg, point, "base_link");
    }else{
      point = msg;
    }
    nao_interaction_msgs::TrackerPointAt r;
    nao_interaction_msgs::MotionSetAngles fingers;
    if (_side == pepper_arm_manager::LEFT) {
      r.request.effector =
          nao_interaction_msgs::TrackerPointAtRequest::EFFECTOR_LARM;
      fingers.request.names = {"LHand", "LWristYaw"};
      fingers.request.angles = {1.0, -1.8151424220741028};
    } else {
      r.request.effector =
          nao_interaction_msgs::TrackerPointAtRequest::EFFECTOR_RARM;
      fingers.request.names = {"RHand", "RWristYaw"};
      fingers.request.angles = {1.0, 1.8151424220741028};
    }
    fingers.request.max_speed_fraction = 1.0;
    r.request.target.x = point.point.x;
    r.request.target.y = point.point.y;
    r.request.target.z = point.point.z;
    r.request.frame = nao_interaction_msgs::TrackerPointAtRequest::FRAME_TORSO;
    float speed = std::max(std::min(ARM_MAX_SPEED, _nh->param(ARM_SPEED_PARAM, ARM_MIN_SPEED)), ARM_MIN_SPEED);
    r.request.max_speed_fraction = speed;
    _pointAtSrv.call(r);
    _setAnglesSrv.call(fingers);
    done();
  }
}

