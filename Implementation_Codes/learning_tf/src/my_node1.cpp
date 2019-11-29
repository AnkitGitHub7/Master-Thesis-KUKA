/* This source file provides the implementation for monitoring the position of the lift_link. 
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


int main(int argc, char **argv)
{
  /*Initalizing the ROS and PoseUpdate is the name of the ROS node */
  ros::init(argc, argv, "PoseUpdate1");
  /*Creation of ROS handle*/
  ros::NodeHandle n;
   /*Creation of ROS publisher to publish the lifting information on chatter1 topic*/
  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Twist>("chatter1", 1000);
  /*Creation of ROS tranform listener to store the transformations*/
  tf::TransformListener listener;
    /*Initalizing the ROS loop rate*/
  ros::Rate rate(150.0);
  
  while (n.ok())
  {
    tf::StampedTransform transform;
    geometry_msgs::Twist msg;
      /*Implementation for try-catch block*/
    try
    {

        ros::Time now = ros::Time::now();
        /*Call to method lookupTransform to get the transformation information between the lift_link and base_link coordinate frames*/
        listener.lookupTransform("/lift_link","/base_link",ros::Time(0), transform);
        ROS_INFO("Got a transform! x = %f, y = %f and theta = %f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getRotation().z());
        /*Copying the data into other variable msg*/
	      msg.linear.x= transform.getOrigin().x();
	      msg.linear.y= transform.getOrigin().y();
        msg.linear.z= transform.getOrigin().z();
        msg.angular.x= transform.getRotation().x();
        msg.angular.y= transform.getRotation().y();
	      msg.angular.z= transform.getRotation().z();

        ROS_INFO("Got a transform! x = %f, y = %f and z = %f",transform.getOrigin().x(),transform.getOrigin().y(),msg.linear.z);
        /*Publishing the variable msg with lift_link position information on the ROS topic chatter_pub1*/
	      chatter_pub1.publish(msg);
    }

    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    } 

    rate.sleep();

  }
  ROS_ERROR("I DIED");
  /*End of main*/
  return 0;
}
