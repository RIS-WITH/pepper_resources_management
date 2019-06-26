#ifndef pepper_arm_manager_ARTIFICIALLIFE_H
#define pepper_arm_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"
#include "pepper_arm_manager/Side.h"

namespace pepper_arm_manager {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer, const pepper_arm_manager::eSide nh);

private:
  virtual void inLoop();
  virtual void init();

  ros::ServiceClient _restSrv;
  pepper_arm_manager::eSide _side;
};

} // namespace pepper_arm_manager

#endif // pepper_arm_manager_ARTIFICIALLIFE_H
