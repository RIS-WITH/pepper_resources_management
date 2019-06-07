#include "pepper_head_manager_msgs/StateMachineRegister.h"
#include "pepper_head_manager_msgs/StateMachineExtract.h"
#include "pepper_head_manager_msgs/PrioritizedJointTrajectory.h"
#include "pepper_head_manager_msgs/PrioritizedPoint.h"
#include "pepper_head_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class PepperHeadManager : public resource_management::ResourceManager<pepper_head_manager_msgs::StateMachineRegister
      ,pepper_head_manager_msgs::StateMachineExtract
      ,pepper_head_manager_msgs::PrioritizedJointTrajectory
      ,pepper_head_manager_msgs::PrioritizedPoint
>
{
public:
    PepperHeadManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"human_monitoring", "env_monitoring", "speaking", "acting"}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedJointTrajectoryMsg(data, is_new); });
        resource_management::MessageWrapper<geometry_msgs::PointStamped>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedPointMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<pepper_head_manager::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pepper_head_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pepper_head_manager_msgs::StateMachine &msg) override;
    pepper_head_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPrioritizedJointTrajectoryMsg(naoqi_bridge_msgs::JointAngleTrajectory msg, bool is_new);
    void publishPrioritizedPointMsg(geometry_msgs::PointStamped msg, bool is_new);
};
