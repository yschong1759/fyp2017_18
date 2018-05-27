#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher odom_rpy_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::Pose2D pose2d;
  pose2d.x = msg->pose.pose.position.x;
  pose2d.y = msg->pose.pose.position.y;

  tf::Quaternion q(
  msg->pose.pose.orientation.x,
  msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z,
  msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose2d.theta = yaw;
  odom_rpy_pub.publish(pose2d);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_rpy");
  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

  odom_rpy_pub = n.advertise<geometry_msgs::Pose2D>("odom_to_rpy", 10);

  ros::spin();
  return 0;
}

