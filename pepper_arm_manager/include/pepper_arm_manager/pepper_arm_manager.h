#include "pepper_arm_manager/ArtificialLife.h"
#include "pepper_arm_manager/Side.h"
#include "pepper_arm_manager_msgs/PrioritizedJointTrajectory.h"
#include "pepper_arm_manager_msgs/PrioritizedPoint.h"
#include "pepper_arm_manager_msgs/StateMachineExtract.h"
#include "pepper_arm_manager_msgs/StateMachineRegister.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>
#include <ros/service_client.h>

#include <nao_interaction_msgs/TrackerPointAt.h>
#include <nao_interaction_msgs/MotionSetAngles.h>

#include <tf2_ros/transform_listener.h>

#define ARM_SPEED_PARAM "arm_speed"
#define ARM_MIN_SPEED 0.2
#define ARM_MAX_SPEED 1.



class PepperArmManager : public resource_management::ResourceManager<pepper_arm_manager_msgs::StateMachineRegister
      ,pepper_arm_manager_msgs::StateMachineExtract
      ,pepper_arm_manager_msgs::PrioritizedJointTrajectory
      ,pepper_arm_manager_msgs::PrioritizedPoint
>
{


public:
    PepperArmManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false, pepper_arm_manager::eSide side=pepper_arm_manager::LEFT):
        ResourceManager (std::move(nh),{"manipulation", "social"}, plugins, synchronized), _side(side), _tfListener(_tfBuffer)
    {
        nh->setParam(ARM_SPEED_PARAM, ARM_MIN_SPEED);

        _pointAtSrv = nh->serviceClient<nao_interaction_msgs::TrackerPointAt>("/naoqi_driver/tracker/point_at");
        _setAnglesSrv = nh->serviceClient<nao_interaction_msgs::MotionSetAngles>("/naoqi_driver/motion/angle_interpolation_with_speed");

        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedJointTrajectoryMsg(data, is_new); });
        resource_management::MessageWrapper<geometry_msgs::PointStamped>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedPointMsg(data, is_new);});

        // Remove if your do not need artificial life
        _artificialLife = std::make_shared<pepper_arm_manager::ArtificialLife>(_artificialLifeBuffer, side);

    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pepper_arm_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pepper_arm_manager_msgs::StateMachine &msg) override;
    pepper_arm_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPrioritizedJointTrajectoryMsg(naoqi_bridge_msgs::JointAngleTrajectory msg, bool is_new);
    void publishPrioritizedPointMsg(geometry_msgs::PointStamped msg, bool is_new);

    ros::ServiceClient _pointAtSrv;
    ros::ServiceClient _setAnglesSrv;

    pepper_arm_manager::eSide _side;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

};
