#include "pepper_arm_manager/pepper_arm_manager.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"pepper_arm_manager");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    PepperArmManager mgr(nh, plugins, true);

    std::thread th(&PepperArmManager::run, &mgr);

    ros::spin();

    th.join();
}
