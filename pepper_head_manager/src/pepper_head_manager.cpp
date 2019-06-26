#include "pepper_head_manager/pepper_head_manager.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  if (is_new){
    _previousPoint = nullptr;
    nao_interaction_msgs::MotionSetAnglesRequest req;
    nao_interaction_msgs::MotionSetAngles r;
    for (size_t i=0; i < msg.joint_names.size(); i++){
      req.names.push_back(msg.joint_names[i]);
      req.angles.push_back(msg.joint_angles[i]);
    }
    float speed = std::max(std::min(HEAD_MAX_SPEED, _nh->param(HEAD_SPEED_PARAM, HEAD_MIN_SPEED)), HEAD_MIN_SPEED);
    req.max_speed_fraction = speed;
    r.request = req;
    _setAnglesSrv.call(r);
  }
}

void PepperHeadManager::publishPrioritizedPointMsg(geometry_msgs::PointStamped msg, bool is_new)
{
    geometry_msgs::PointStamped point;
    try {
      auto t = _tfBuffer.lookupTransform("base_link", msg.header.frame_id, ros::Time(0));
      tf2::doTransform(msg, point, t);
    }catch (tf2::TransformException &transformException){
      ROS_WARN("Failed to transform frame '%s' in base_link frame: %s", msg.header.frame_id.c_str(), transformException.what());
      return;
    }

    float d;
    if (_previousPoint != nullptr) {
      d = std::hypot(std::hypot(point.point.x - _previousPoint->x,
                                      point.point.y - _previousPoint->y),
                           point.point.z - _previousPoint->z);
    }
    if (is_new || _previousPoint == nullptr || d > 0.1) {
      float speed = std::max(std::min(HEAD_MAX_SPEED, _nh->param(HEAD_SPEED_PARAM, HEAD_MIN_SPEED)), HEAD_MIN_SPEED);
      _previousPoint = boost::make_shared<geometry_msgs::Point>(point.point);
      nao_interaction_msgs::TrackerLookAt r;
      //_tfBuffer.transform<geometry_msgs::PointStamped>(msg, point, "/base_link"); point = msg.point;
      r.request.use_whole_body = false;
      r.request.max_speed_fraction = speed;
      r.request.target.x = point.point.x;
      r.request.target.y = point.point.y;
      r.request.target.z = point.point.z;
      r.request.frame = nao_interaction_msgs::TrackerLookAtRequest::FRAME_TORSO;
      _lookAtSrv.call(r);
      done();
    }

}

