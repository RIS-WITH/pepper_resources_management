#include "pepper_arm_manager/pepper_arm_manager.h"

int main(int argc, char *argv[]){

  pepper_arm_manager::eSide side;

  if (std::string(argv[1]) == "left"){
    side = pepper_arm_manager::LEFT;
    ros::init(argc,argv,"pepper_arm_manager_left");
  }else if (std::string(argv[1]) == "right"){
    side = pepper_arm_manager::RIGHT;
    ros::init(argc,argv,"pepper_arm_manager_right");
  }else{
    throw std::runtime_error(
        R"(pr2_arm_manager needs to be launched with either "left" or "right" as first command line argument.)");
  }
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));


    std::vector<std::string> plugins;
    for(int i = 2; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    PepperArmManager mgr(nh, plugins, side);

    std::thread th(&PepperArmManager::run, &mgr);

    ros::spin();

    th.join();
}
