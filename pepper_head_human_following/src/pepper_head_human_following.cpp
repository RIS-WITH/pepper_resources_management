#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <pepper_head_manager_msgs/PrioritizedPoint.h>
#include <resource_management_msgs/MessagePriority.h>


class HumanFollower {
public:
  HumanFollower(ros::NodeHandle& nh): tfListener(tfBuffer){

    subscriber_ = nh.subscribe<const std_msgs::String&>("human_to_look", 10, &HumanFollower::onNewHumanToTrack, this);
    publisher_ = nh.advertise<pepper_head_manager_msgs::PrioritizedPoint>(
        "/pepper_head_manager/human_monitoring/pepper_head_manager_msgs1_PrioritizedPoint", 10);
    t_ = nh.createTimer(ros::Duration(0.3), &HumanFollower::onTimer, this);
    ROS_INFO("Pepper head human follower started.");

  }

  void onNewHumanToTrack(const std_msgs::String& human){
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
      if (tfBuffer.canTransform(frameToTrack, "map", ros::Time(0))) {
        auto transform =
            tfBuffer.lookupTransform("map", frameToTrack, ros::Time(0));
        if (ros::Time::now() - transform.header.stamp <= ros::Duration(0.5)) {
          point_with_priority.priority.value =
              resource_management_msgs::MessagePriority::URGENT;

          double d;
          if (lastTransform.header.frame_id != "") {
            d = hypot(hypot(transform.transform.translation.x -
                                   lastTransform.transform.translation.x,
                                   transform.transform.translation.y -
                                   lastTransform.transform.translation.y),
                             transform.transform.translation.z -
                             lastTransform.transform.translation.z);
          }
          if (lastTransform.header.frame_id == "" || d >= 0.2){
            lastTransform = transform;
          }
          point_with_priority.data.header.stamp = ros::Time::now();
          point_with_priority.data.header.frame_id = "map";
          point_with_priority.data.point.x = lastTransform.transform.translation.x;
          point_with_priority.data.point.y = lastTransform.transform.translation.y;
          point_with_priority.data.point.z = lastTransform.transform.translation.z;
        }
      }
    }
    if (point_with_priority.priority.value == resource_management_msgs::MessagePriority::VOID){
      lastTransform = geometry_msgs::TransformStamped();
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
  geometry_msgs::TransformStamped lastTransform;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "human_point_publisher");
  ros::NodeHandle nh("~");

  HumanFollower hf(nh);

  ros::spin();

}

