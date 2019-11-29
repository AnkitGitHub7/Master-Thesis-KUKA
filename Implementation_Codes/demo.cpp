/* This source file provides the implementation for the spline tracjectory. 
 * It uses the input reference points to get the interpolated trajectory information. 
 * The use number of points per segment, end condition and the middle condition as arguments for the constructor.
*/

/*
 * Author: Ankit Ankit
 */

#include <tf/tf.h>

//int k1=0;

int main(int argc, char** argv)
{
      ros::init(argc, argv, "path_smoothing_ros_demo");
      ros::NodeHandle nh("~");
      ROS_INFO_STREAM("Namespace:" << nh.getNamespace());
      /*refreshing frequency period*/
      ros::Rate loop_rate(50); 
      /*Variable to store the pose components of the mobile robot*/
      geometry_msgs::PoseStamped motion1;
      /*ROS publishers to publish the information about the intial pose, final pose, inital path and the smoothed path*/
      ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
      ros::Publisher finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
      ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("initial_path", 1, true);
      ros::Publisher smoothedPathPub = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);
      /*ROS publisher to publish the motion of the robot on the /move_base_simple/goal topic*/
      ros::Publisher pub1 = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,true);

      int pointsPerUnit, skipPoints;
      bool useEndConditions, useMiddleConditions;
      /*Initalization of the configuration parameters*/
      nh.param<int>("points_per_unit", pointsPerUnit, 5);
      nh.param<int>("skip_points", skipPoints, 0);
      nh.param<bool>("use_end_conditions", useEndConditions, false);
      nh.param<bool>("use_middle_conditions", useMiddleConditions, false);
      /*Retrieving the information about the input poses of the robot*/
      XmlRpc::XmlRpcValue poseList;
      if (!nh.getParam("path_poses", poseList))
      {
        ROS_FATAL("Failed to load path point list");
        exit(EXIT_FAILURE);
      }
      /*variables to store the pose of the updated path*/
      nav_msgs::Path path, smoothedPath;
      path.header.frame_id = "map";
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      /*Parsing the user defined inputs for position x,y and the orientation*/
      for (int i = 0; i < poseList.size(); i++)
      {
        pose.pose.position.x = static_cast<double>(poseList[i]["x"]);
        pose.pose.position.y = static_cast<double>(poseList[i]["y"]);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseList[i]["yaw"]);
        path.poses.push_back(pose);
      }

      /* creating cubic spline interpolator constructor with the with the input
      * arguments pointsPerUnit, skipPoints, useEndConditions and useMiddleConditions.
      */
      path_smoothing::CubicSplineInterpolator csi(800, 0, 1, 0	);
      /*Calling the interpolatePath funnction on the above created class object*/
      csi.interpolatePath(path, smoothedPath);

      ROS_INFO("POINTS PER SEGMENT IS %i",pointsPerUnit);
      /*Publishing the path poses information */
      initialPosePub.publish(path.poses.front());
      finalPosePub.publish(path.poses.back());
      pathPub.publish(path);
      smoothedPathPub.publish(smoothedPath);

      /* Looping for 5 times */

      for (int k1 = 0; k1 < 5; k1++)
      {

        for (int k = 0; k < smoothedPath.poses.size(); k=k+10)
        {

            ROS_INFO("Data are: %f and %f" , smoothedPath.poses[k].pose.position.x,smoothedPath.poses[k].pose.position.y);
            /* Filling the position and orientation information to the motion1 variable */
            motion1.header.frame_id = "/map";
            motion1.header.stamp = ros::Time::now();
            motion1.pose.position.x= (smoothedPath.poses[k].pose.position.x)/5;
            motion1.pose.position.y=  (smoothedPath.poses[k].pose.position.y)/5;
            motion1.pose.position.z= 0;
            motion1.pose.orientation.x=  0;
            motion1.pose.orientation.y= 0;
            motion1.pose.orientation.z=  0;
            motion1.pose.orientation.w=  1;
            /* Publishing motion on simple goal topic */
            pub1.publish(motion1);
            loop_rate.sleep();
        }

      ROS_INFO("VALUE OF K1 is: %i" ,k1);

      }

      ros::Time currTime = ros::Time::now();
      while (ros::ok() && ros::Time::now().toSec() - currTime.toSec() < 2.0)
      {
        ros::spinOnce();
      }

      return 0;
	

}
