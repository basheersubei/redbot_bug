/* Copyright 2015, Basheer Subei
 * ROS Node that translates aruco Transform messages to Pose2D messages.
 */
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include "ros/ros.h"

ros::Publisher odom_publisher;

void transform_callback(const geometry_msgs::TransformStamped& t) {
  geometry_msgs::Pose2D odom;
  odom.x = t.transform.translation.x;
  odom.y = t.transform.translation.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(t.transform.rotation, q);
  odom.theta = tf::getYaw(q);

  odom_publisher.publish(odom);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle n;
  odom_publisher = n.advertise<geometry_msgs::Pose2D>("odom", 10);
  ros::Subscriber sub = n.subscribe("ar_single_board/transform", 1000, transform_callback);

  ros::spin();

  return 0;
}
