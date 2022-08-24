#include "pepper_head_manager/ArtificialLife.h"
#include "pepper_head_manager_msgs/PrioritizedPoint.h"
#include "pepper_head_manager_msgs/PrioritizedJointTrajectory.h"
#include "resource_management/message_storage/MessageWrapper.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <naoqi_bridge_msgs/JointAngleTrajectory.h>

namespace pepper_head_manager {

ArtificialLife::ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer) :
        resource_management::ArtificialLife(100 /* you can change the artficial life frame rate here*/, buffer),
        poisson_distribution(5000), normal_distribution(0.0, 0.2)
{
  // set an initial value in the artificial life buffer
  // if you do not do that that resource will not start in artificial life mode

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  // auto wrapped_prioritized_point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_prioritized_point_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_prioritized_point_data);

  geometry_msgs::PointStamped data;
  data.header.frame_id = "base_link";
  data.point.x = 5.0;
  data.point.y = 0.0;
  data.point.z = 1.1;
  auto wrapped_prioritized_point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  wrapped_prioritized_point_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_prioritized_point_data);
}

void ArtificialLife::init()
{
  // Put our own initialisation function here
  // It will be called each time a new artificial life cycle begins

  // Set an initial value in the artificial life buffer
  // if you do not do that, at each new cycle, the resource
  // will start with the previous artificial life value

  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  // auto wrapped_prioritized_point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_prioritized_point_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_prioritized_point_data);
  geometry_msgs::PointStamped data;
  data.header.frame_id = "base_link";
  data.point.x = 5.0;
  data.point.y = 0.0;
  data.point.z = 1.1;
  auto wrapped_prioritized_point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  wrapped_prioritized_point_data->setPriority(resource_management::low);
  _buffer->setData(wrapped_prioritized_point_data);
}

void ArtificialLife::inLoop()
{
  // This function will be called at the specified frame rate
  // will the artificial life cycle is running

  // DO NOT CREATE YOUR OWN LOOP HERE

  // creat a new data and feed it to the artficial life buffer
  // Example:

  // 1 - Wrap your data with one of your types:
  // auto wrapped_PrioritizedJointTrajectory_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(data);
  // auto wrapped_prioritized_point_data = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(data);
  //
  // 2 - Set the useless priority to your wrapped data:
  // wrapped_PrioritizedJointTrajectory_data->setPriority(resource_management::useless);
  // wrapped_prioritized_point_data->setPriority(resource_management::useless);
  //
  // 3 - Insert your wrapped data into the rtificial life buffer:
  // _buffer->setData(wrapped_PrioritizedJointTrajectory_data);
  // _buffer->setData(wrapped_prioritized_point_data);

  auto now = std::chrono::system_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_moved_time).count() >= next_duration) 
  {
    // Calcul of the locate at point 
    double x = 5; 
    double y = normal_distribution(generator)*2.5;
    double z = 1.1 + normal_distribution(generator)*2.5;
    // transform the point to angle joint 
    double z_angle = std::atan2(z,x);
    double y_angle = std::atan2(y,x);
    double mvt_duration = poisson_distribution(generator)/2;

    naoqi_bridge_msgs::JointAngleTrajectory data_traj;
    data_traj.joint_names.push_back("HeadPitch");
    data_traj.joint_angles.push_back(-z_angle);
    data_traj.times.push_back(mvt_duration);

    data_traj.joint_names.push_back("HeadYaw");
    data_traj.joint_angles.push_back(y_angle);
    data_traj.times.push_back(mvt_duration);

    auto wrapped_pitch_yaw_data = std::make_shared<resource_management::MessageWrapper<naoqi_bridge_msgs::JointAngleTrajectory>>(
            data_traj);
    wrapped_pitch_yaw_data->setPriority(resource_management::low);
    _buffer->setData(wrapped_pitch_yaw_data);

    if(normal_distribution(generator)>0)
      next_duration = poisson_distribution(generator)*1.5;
    else
      next_duration = poisson_distribution(generator);
    last_moved_time = now;
  }
}

} // namespace pepper_head_manager
