// c++
#include <vector>
#include <cstdlib>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

// ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pov_camera_angle_rotation");
    ros::NodeHandle nh;

    int Hz;
    int direction_number;

    nh.param("loop_rate", Hz, 10);                      //loop_rate (defalt 10Hz)
    nh.param("direction_number", direction_number, 6);  //direction number (default 6)
    
    ros::Rate loop_rate(Hz);

    tf::TransformBroadcaster tf_broadcaster_;
    std::string frame = "camera/flipped_front";
    std::string child_frame = "flipped_pov_camera";
    int count = 0;

    tf::Quaternion tf_quaternion;
    double roll = 0;
    double pitch = 0;
    double yaw   = (double)count * M_PI / (double)direction_number; 
    tf_quaternion.setRPY(roll, pitch, yaw);

    while(ros::ok()){
        ros::spin();

        tf_broadcaster_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf_quaternion, tf::Vector3(0.0, 0.0, 0.0)),
                ros::Time::now(),
                frame,
                child_frame
            )
        );

        count++;
        if(count == direction_number)count = 0;

        loop_rate.sleep();
    }
}
