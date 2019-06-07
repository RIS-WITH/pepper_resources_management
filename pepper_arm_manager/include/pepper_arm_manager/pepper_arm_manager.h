#include "pepper_arm_manager_msgs/StateMachineRegister.h"
#include "pepper_arm_manager_msgs/StateMachineExtract.h"
#include "pepper_arm_manager_msgs/PrioritizedJointTrajectory.h"
#include "pepper_arm_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class PepperArmManager : public resource_management::ResourceManager<pepper_arm_manager_msgs::StateMachineRegister
      ,pepper_arm_manager_msgs::StateMachineExtract
      ,pepper_arm_manager_msgs::PrioritizedJointTrajectory
>
{
public:
    PepperArmManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"manipulation", "social"}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedJointTrajectoryMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<pepper_arm_manager::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pepper_arm_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pepper_arm_manager_msgs::StateMachine &msg) override;
    pepper_arm_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPrioritizedJointTrajectoryMsg(naoqi_bridge_msgs::JointAngleTrajectory msg, bool is_new);
};
