#include "pepper_head_manager/pepper_head_manager.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"pepper_head_manager");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    PepperHeadManager mgr(nh, plugins);

    std::thread th(&PepperHeadManager::run, &mgr);

    ros::spin();

    th.join();
}
