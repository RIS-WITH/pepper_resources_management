#include "pepper_base_manager_msgs/StateMachineRegister.h"
#include "pepper_base_manager_msgs/StateMachineExtract.h"
#include "pepper_base_manager_msgs/PrioritizedTwist.h"
#include "pepper_base_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>

#include <thread>

class PepperBaseManager : public resource_management::ResourceManager<pepper_base_manager_msgs::StateMachineRegister
      ,pepper_base_manager_msgs::StateMachineExtract
      ,pepper_base_manager_msgs::PrioritizedTwist
>
{
public:
    PepperBaseManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"acting", "exploring"}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<geometry_msgs::Twist>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedTwistMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<pepper_base_manager::ArtificialLife>(_artificialLifeBuffer));
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const pepper_base_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const pepper_base_manager_msgs::StateMachine &msg) override;
    pepper_base_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPrioritizedTwistMsg(geometry_msgs::Twist msg, bool is_new);
};
