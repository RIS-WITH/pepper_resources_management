#ifndef pepper_arm_manager_ARTIFICIALLIFE_H
#define pepper_arm_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"

namespace pepper_arm_manager {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();
};

} // namespace pepper_arm_manager

#endif // pepper_arm_manager_ARTIFICIALLIFE_H
