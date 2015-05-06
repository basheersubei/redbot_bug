/*
 * rosserial node subscribes to transform data from aruco marker
 * and uses that as odometry source for running bug algorithm (RedBot)
 */
//#define USE_USBCON
#include <ros.h>
#include <tf/tf.h>
#include <ros/time.h>
#include <RedBot.h>

// Instantiate the motor control class. This only needs to be done once
//  and indeed SHOULD only be done once!
RedBotMotors motor;
// Instantiate the sensors. Sensors can only be created for analog input
//  pins; the Xbee software serial uses pins A0 and A1 and the
//  accelerometer uses pins A4 and A5.
RedBotSensor lSen = RedBotSensor(A3);
RedBotSensor cSen = RedBotSensor(A6);
RedBotSensor rSen = RedBotSensor(A7);
// Variable used to wait on a bump before starting operation.
boolean following = false;



ros::NodeHandle  nh;

void transform_callback( const geometry_msgs::TransformStamped& t);
ros::Subscriber<geometry_msgs::TransformStamped> transform_sub("/ar_single_board/transform", transform_callback );
//ros::Publisher odom_pub("odom_debug", &odom);

void transform_callback( const geometry_msgs::TransformStamped& t){
//  nav_msgs::Odometry odom;
//  odom.child_frame_id = "redbot";
//  odom.pose.pose.position.x = t.transform.translation.x;
//  odom.pose.pose.position.y = t.transform.translation.y;
//  odom.pose.pose.orientation = tf::createQuaternionFromYaw(t.transform.rotation.z);
//  
  following = true;
}

void setup()
{
  nh.initNode();            // initializes ROS node
  nh.subscribe(transform_sub);
//  nh.advertise(odom_pub);
}

void loop()
{
  if (following)
  {
    followNoCal();
  }  
}
