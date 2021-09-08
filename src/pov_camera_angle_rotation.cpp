#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BroadCasterTest
{
public:
  BroadCasterTest() : nh_()
  {
    timer_ = nh_.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& e) {
      broadcast_dynamic_tf();
      counter_++;
    });
  }

private:
  void broadcast_dynamic_tf(void)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera/flipped_front";
    transformStamped.child_frame_id = "flipped_pov_camera";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, counter_ * M_PI / 6);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    dynamic_br_.sendTransform(transformStamped);
  }
  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster dynamic_br_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  int counter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_static_tf2_broadcaster");
  BroadCasterTest broadcast_test;
  ros::spin();
  return 0;
};