#ifndef pepper_head_manager_ARTIFICIALLIFE_H
#define pepper_head_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"
#include <random>
#include <chrono>

namespace pepper_head_manager {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer);

  std::default_random_engine generator;
  std::poisson_distribution<int> poisson_distribution;
  std::normal_distribution<double> normal_distribution;
  std::chrono::time_point<std::chrono::system_clock> last_moved_time;
  int next_duration;

private:
  virtual void inLoop();
  virtual void init();
};

} // namespace pepper_head_manager

#endif // pepper_head_manager_ARTIFICIALLIFE_H
