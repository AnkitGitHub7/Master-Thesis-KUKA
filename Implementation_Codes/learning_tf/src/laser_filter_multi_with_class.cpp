/* This source file provides the implementation for laser median filter. 
 * It uses the input from the original laser scan and published new scan with the median on ROS topic /published_topic. 
 * This topic is futher used for the object shape recognition.
*/

/*
 * Author: Ankit Ankit
 */

/*Inclusion of headers*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <list>

using namespace std;

/*Creation of the new class to perform the median implementation*/
class SubscribeAndPublish
{
  /*Implementing the public method for the class*/
  public:
    SubscribeAndPublish()
    {

      is_initialized = false;
      i=0;
      cnt = 0;
      median_size = 11;

      pub = nh.advertise<sensor_msgs::LaserScan>("/published_topic", 1);

      sub = nh.subscribe("/scan", 1000, &SubscribeAndPublish::counterCallback, this);
    }
  /*ROS subscriber callback to get the information about the original laser scan from /scan topic*/
  void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)      
  {

      new_scan = *msg;
      ROS_INFO("ranges size: %i ", (int)msg->ranges.size()); 
      /*Checking if the vector to be filled is already initialized*/
      if (!is_initialized)
      {
        /*Looping through all the 689 elements of the laser scan array*/
        for (int k = 0; k < msg->ranges.size(); k++)
        {
          /*initializing all the elements to 0*/
          laser_data.push_back(new list<float>());
          new_scan.ranges[k] = 0;
        }
        is_initialized = true;
      }
      /*This loop is store the data in the vector of size 11*/
      for (int k = 0; k < msg->ranges.size(); k++)
      {
          laser_data[k]->push_back(msg->ranges[k]);
          if (laser_data[k]->size() > median_size)
          laser_data[k]->pop_front();
      }
      /*Checking for 10 reading of the laser scans*/
      if (cnt++ % 10 == 0)
      {
        for (int k = 0; k < msg->ranges.size(); k++)   
        {
          std::list<float> current(*laser_data[k]);
          /*Sorting the list containing the median data information*/
          current.sort(); 
          /*Creating an iterator to loop thorugh all the elements of the list*/
          std::list<float>::iterator it = current.begin();
          /*Median calculation by considering the middle elements after sorting*/
          std::advance(it, median_size / 2);
          new_scan.ranges[k] = *it;
          ROS_INFO("k %i -------- %f", k , *it);
        }
          /*Publishing the new laser scan*/
          pub.publish(new_scan);
      }

}
 /*Declaration of the private attributes of the class*/
private:
  /*Defining the handle for the ROS node*/
  ros::NodeHandle nh; 
  /*Defining the publisher and subscriber node*/
  ros::Publisher pub;
  ros::Subscriber sub ;
  /*Variable to store the information about the new laser scan data*/
  sensor_msgs::LaserScan new_scan;
  /*Vector to store the median data*/
  vector<list<float> * > laser_data; 
  /*variable to check the initalization of the new vector with 0*/
  bool is_initialized;
  int i;
  int cnt;
  /*Defining the median size*/
  int median_size;

};/*End of class*/


int main(int argc, char **argv)
{
  /*Initializing the ROS and ros node subscribe_and_publish is passed as an input argument*/
  ros::init(argc, argv, "subscribe_and_publish");
  /*Create an object of class SubscribeAndPublish */
  SubscribeAndPublish SAPObject;
  /*To perform continuous looping*/
  ros::spin();
  /*End of main*/
  return 0;
}