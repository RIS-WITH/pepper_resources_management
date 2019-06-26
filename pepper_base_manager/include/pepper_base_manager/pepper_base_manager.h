#include "pepper_base_manager_msgs/StateMachineRegister.h"
#include "pepper_base_manager_msgs/StateMachineExtract.h"
#include "pepper_base_manager_msgs/PrioritizedTwist.h"
#include "pepper_base_manager_msgs/PrioritizedAngle.h"
#include "pepper_base_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <nao_interaction_msgs/GoToPose.h>

#include <thread>

#define CMD_VEL_TOPIC "/cmd_vel"

class PepperBaseManager : public resource_management::ResourceManager<pepper_base_manager_msgs::StateMachineRegister
      ,pepper_base_manager_msgs::StateMachineExtract
      ,pepper_base_manager_msgs::PrioritizedTwist
      ,pepper_base_manager_msgs::PrioritizedAngle
>
{
public:
    PepperBaseManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"acting", "exploring", "navigating"}, plugins, synchronized)
    {
        _moveToSrv = nh->serviceClient<nao_interaction_msgs::GoToPose>("/naoqi_driver/motion/move_to");
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<geometry_msgs::Twist>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedTwistMsg(data, is_new); });
        resource_management::MessageWrapper<float>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedAngleMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<pepper_base_manager::ArtificialLife>(_artificialLifeBuffer));

        _cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 1);
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pepper_base_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pepper_base_manager_msgs::StateMachine &msg) override;
    pepper_base_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPrioritizedTwistMsg(geometry_msgs::Twist msg, bool is_new);
    void publishPrioritizedAngleMsg(float msg, bool is_new);

    ros::Publisher _cmd_vel_pub;
    ros::ServiceClient _moveToSrv;
};
