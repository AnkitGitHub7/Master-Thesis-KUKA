/* This source file provides the implementation for monitoring the position and the orientation of the mobile robot. 
 * It uses the llok up transform to get the information regarding the pose of the base_link in the map coordinate frame. 
 * This data is published and can be used by the other nodes who need to asses the pose of the robot in the map frame.
*/

/*
 * Author: Ankit Ankit
 */

/*Inclusion of headers*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>


int main(int argc, char **argv)
{
  /*Initalizing the ROS and PoseUpdate is the name of the ROS node */
  ros::init(argc, argv, "PoseUpdate");
  /*Creation of ROS handle*/
  ros::NodeHandle n;
  /*Creation of ROS publisher to publish the pose information on chatter topic*/
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);
  /*Creation of ROS tranform listener to store the transformations*/
  tf::TransformListener listener;
  /*Initalizing the ROS loop rate*/
  ros::Rate rate(40.0);
  std::ofstream myfile;
  myfile.open("/tmp/OUTPUTXY.txt");

  while (n.ok())
  {
    tf::StampedTransform transform;
    geometry_msgs::Twist msg;
    /*Implementation for try-catch block*/
    try
    {
      /*Call to method lookupTransform to get the transformation information between the map and base_link coordinate frames*/
      listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
      myfile << transform.getOrigin().x() << "," << transform.getOrigin().y() << "\n";
      /*Copying the position data into other variable msg*/
      msg.linear.x= transform.getOrigin().x();
      msg.linear.y= transform.getOrigin().y();
      msg.linear.z= transform.getOrigin().z();
      /*Creating the quaternion from euler angles to avoid the gimble lock*/
    	tf::Quaternion q(
      transform.getRotation().x(),
      transform.getRotation().y(),
      transform.getRotation().z(),
      transform.getRotation().w());
      tf::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
      /*Copying the orientation data into other variable msg*/
      msg.angular.x= roll;
      msg.angular.y= pitch;
      msg.angular.z= yaw;

      ROS_INFO("Got a transform! x = %f, y = %f and theta yaw = %f",transform.getOrigin().x(),transform.getOrigin().y(),yaw);
      /*Publishing the variable msg with pose information on the ROS topic chatter*/
      chatter_pub.publish(msg);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Nope! %s", ex.what());
        } 
      rate.sleep();
  }
  myfile.close();
  ROS_ERROR("I DIED");
  /*End of main*/
  return 0;
}
