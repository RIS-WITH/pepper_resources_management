#include "pepper_base_manager/pepper_base_manager.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"pepper_base_manager");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    PepperBaseManager mgr(nh, plugins, true);

    std::thread th(&PepperBaseManager::run, &mgr);

    ros::spin();

    th.join();
}
