#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BroadCasterTest
{
public:

  std::string param_pov_camera_frame_;
  std::string param_camera_frame_;
  double loop_time_;
  int direction_number_;

  BroadCasterTest() : nh_()
  {
    nh_.param("pov_camera_frame", param_pov_camera_frame_, std::string("flipped_pov_camera")); //カメラの視点方向のtf（子フレーム）
    nh_.param("camera_frame", param_camera_frame_, std::string("camera/flipped_front"));       //カメラの位置のtf（親フレーム）
    nh_.param("loop_time", loop_time_, 0.1);                                                   //tf更新する時間間隔[s]
    nh_.param("direction_number", direction_number_, 6);                                       //視点の方向数

    timer_ = nh_.createTimer(ros::Duration(loop_time_), [&](const ros::TimerEvent& e) {
      broadcast_dynamic_tf();
      counter_++;
    });
  }

private:
  void broadcast_dynamic_tf(void)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = param_camera_frame_;
    transformStamped.child_frame_id = param_pov_camera_frame_;
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, counter_ * M_PI / direction_number_);
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