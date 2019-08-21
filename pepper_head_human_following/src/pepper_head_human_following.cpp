#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <pepper_head_manager_msgs/PrioritizedPoint.h>
#include <resource_management_msgs/MessagePriority.h>


class HumanFollower {
public:
  HumanFollower(ros::NodeHandle& nh): tfListener(tfBuffer){

    subscriber_ = nh.subscribe<std_msgs::String>("human_to_look", 10, &HumanFollower::onNewHumanToTrack, this);
    publisher_ = nh.advertise<pepper_head_manager_msgs::PrioritizedPoint>(
        "/pepper_head_manager/human_monitoring/pepper_head_manager_msgs1_PrioritizedPoint", 10);
    t_ = nh.createTimer(ros::Duration(0.5), &HumanFollower::onTimer, this);

  }

  void onNewHumanToTrack(std_msgs::String human){
    frameToTrack = human.data;
  }

  void onTimer(const ros::TimerEvent& e){
    std_msgs::Header header;
    header.frame_id = frameToTrack;
    header.stamp = ros::Time::now();
    pepper_head_manager_msgs::PrioritizedPoint point_with_priority;
    geometry_msgs::PointStamped point_stamped;
    point_stamped.header = header;
    point_with_priority.data = point_stamped;
    point_with_priority.priority.value = resource_management_msgs::MessagePriority::VOID;
    if (frameToTrack != "")
    {
      if (tfBuffer.canTransform(frameToTrack, "/base_link", ros::Time(0))) {
        point_with_priority.priority.value =
            resource_management_msgs::MessagePriority::URGENT;
      }
    }
    publisher_.publish(point_with_priority);
  }

protected:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::string frameToTrack;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::Timer t_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pepper_head_pepper_follower");
  ros::NodeHandle nh("~");

  HumanFollower hf(nh);

  ros::spin();

}

