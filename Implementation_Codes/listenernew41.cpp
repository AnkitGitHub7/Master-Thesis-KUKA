/* This source file provides the implementation for the docking application. 
 * It uses the obstacles topic to get the information about the object position. 
 * This data are further used to perform detection of the trolley base.
 * Once the trolley is been recognized, the transformation frames are published in the map frame
 * The platform tis then commanded to move towards the trolley so perform docking
 * Once the platform is under the trolley, it can lift the trolley and move to some distance with the trolley
 * Once the destination is reached, the platform can be lifted down.
 * This explains the complete docking application
*/

/*
 * Author: Ankit Ankit
 */

/*Inclusion for the header files*/
#include "ros/ros.h"
#include <cmath> 
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <fstream>

/*Inclusion for the namespace*/
using namespace std;
using namespace obstacle_detector;

/*Variable to store the information about the robot position*/
geometry_msgs::Twist position_data;
/*Variable to store the information about the platform lift position*/
geometry_msgs::Twist position_data_lift;
/*Boolean variables to perform the condition checks*/
bool val=false;
bool val11=false;
/*Variable to store the count for number of obsjects detected*/
int r=0;
/*Variable to store the distance of robot from the detected object*/
float dis_final2 = 0;
float old_val33 = 50000,new_val33;
float old_val = 500,new_val;
/*Variable to store the coordinates of the detected corners of the trolley*/
float dist,midpoint_x,midpoint_y,orientation_z;
float final_pos_x=0,final_pos_y=0;
float final_pos_x_far=0,final_pos_y_far=0;
float diagonal_mid_x=0,diagonal_mid_y=0;
/*Variable to store all the detected diagonal coordinates*/
vector<float> dia_xi_acc; 
vector<float> dia_yi_acc; 
vector<float> dia_xj_acc; 
vector<float> dia_yj_acc; 
vector<float> midxx; 
vector<float> midyy; 
/*Variable to store the mid point coordinates*/
float midxxavg=0,midyyavg=0;
/*Variable to store the average value of the coordinates*/
float avg_xi=0,avg_yi=0,avg_xj=0,avg_yj=0;
vector<float> cen_xi_acc; 
vector<float> cen_yi_acc; 
vector<float> cen_xj_acc; 
vector<float> cen_yj_acc; 
float avgc_xi=0,avgc_yi=0,avgc_xj=0,avgc_yj=0;

/* ROS Subscriber callback to get the position and orientation of the robot.
 * This is used to monitor the position of the robot in the map coordinate frame*/
void counterCallback1(const geometry_msgs::Twist::ConstPtr& msg1)                                                               
{

  position_data.linear.x = msg1->linear.x;
  position_data.linear.y = msg1->linear.y;
  position_data.angular.z = msg1->angular.z;


}

/* ROS Subscriber callback to get the position data of the platform lift*
 * This is used to monitor the position of the platform lift*/
void counterCallback2(const geometry_msgs::Twist::ConstPtr& msg2)                                                          
{

  position_data_lift.linear.x = msg2->linear.x;
  position_data_lift.linear.y = msg2->linear.y;
  position_data_lift.linear.z = msg2->linear.z;


}

/* ROS Subscriber callback to get the information about the detected objects.
 * This is used to calculated the mid point coordinates of the entry edge of the trolley for the docking application*/
void counterCallback(const obstacle_detector::Obstacles::ConstPtr& msg)                                                 
{
 /*Variables to store the coordinate information about eh detected objects*/
 float cir0_x,cir0_y,cir0_t_rad,cir1_x,cir1_y,cir1_t_rad;    
 float dist;
 final_pos_x=0;
 final_pos_y=0;
 final_pos_x_far=0;
 final_pos_y_far=0;

 /*Variables to store the count of objects detected*/
 int size = msg->circles.size();
 ROS_INFO("Total no of obstacles detected is %i",size);
 float array_centre_x[size] = { }; 
 float array_centre_y[size] = { }; 
 float array_rad[size] = { }; 
 float midpt_x[2] = { }; 
 float midpt_y[2] = { }; 
 float arr_obs_x[4] = { }; 
 float arr_obs_y[4] = { }; 
 float arr_mid_x[2] = { }; 
 float arr_mid_y[2] = { }; 
 /*No of detected entry edge is initialized to zero*/
 r=0;  
 /*Looping through all the detected objects and storing the data*/
 for(int i=0;i< size;i++)
 {

    array_centre_x[i] = msg->circles[i].center.x;
    array_centre_y[i] = msg->circles[i].center.y;
    array_rad[i] = msg->circles[i].true_radius;

 }
 /*Looping through all detected objects*/
 for(int i=0;i< size;i++)
 {  
    /*Verifying the distance and radius check of one detected object with all other remaining objects*/
    for(int j=i+1;j< size;j++)
    {
      /*Calculating the distance between two detected objects*/
      dist = sqrt(pow(array_centre_x[i] - array_centre_x[j], 2) + pow(array_centre_y[i] - array_centre_y[j], 2));
      ROS_INFO("general distance between obstacle %f",dist);
      ROS_INFO("radius are %f  and  %f",array_rad[i],array_rad[j]);
      /*Verifying the distance and the radius checks*/
      if((((dist > 0.80)&&(dist < 0.91))||((dist > 0.62)&&(dist < 0.7)))&&((array_rad[i] < 0.13)&&(array_rad[i] > 0.075))&&((array_rad[j] < 0.13)&&(array_rad[j] > 0.075)))
      {
        /*Diagonal distance check for two detected object pair*/
        if((dist > 0.80)&&(dist < 0.91)){
          /*Accumulating the diagonal detected data*/
          dia_xi_acc.push_back(array_centre_x[i]);  
          dia_yi_acc.push_back(array_centre_y[i]); 
          dia_xj_acc.push_back(array_centre_x[j]);           
          dia_yj_acc.push_back(array_centre_y[j]);   

          ROS_INFO("------- WITHOUT_AVEGARE diagonal points are %f and %f------ ", (array_centre_x[i] + array_centre_x[j])/2, (array_centre_y[i] + array_centre_y[j])/2); 
          /*Averaging the accumulated diagonal detected data*/
          avg_xi = accumulate( dia_xi_acc.begin(), dia_xi_acc.end(), 0.0)/dia_xi_acc.size();   
          avg_yi = accumulate( dia_yi_acc.begin(), dia_yi_acc.end(), 0.0)/dia_yi_acc.size(); 
          avg_xj = accumulate( dia_xj_acc.begin(), dia_xj_acc.end(), 0.0)/dia_xj_acc.size(); 
          avg_yj = accumulate( dia_yj_acc.begin(), dia_yj_acc.end(), 0.0)/dia_yj_acc.size();  

          if(avg_xi != 0)
          {
            /*Calculating the mid point from the diagonal coordinates*/
            diagonal_mid_x = (avg_xi + avg_xj) / 2;
            diagonal_mid_y = (avg_yi + avg_yj) / 2;
            ROS_INFO("------- AVEGARE diagonal points are %f and %f------ ", diagonal_mid_x, diagonal_mid_y);

          }


        }
        /*Check for the distance for the docking entry edge*/
        else if((dist > 0.62)&&(dist < 0.7))
        {
          /*Saving the centre coordinates of first entry edge*/
          arr_obs_x[r]=array_centre_x[i]; 
          arr_obs_y[r]=array_centre_y[i]; 
          ROS_INFO("------obstacle position is postion --%i  --  %f and %f -------", r,arr_obs_x[r], arr_obs_y[r]);
          /*Incrementing the entry edge counter*/
          r++;
          /*Saving the centre coordinates of the second entry edge*/
          arr_obs_x[r]=array_centre_x[j]; 
          arr_obs_y[r]=array_centre_y[j]; 
          /*Calculating the mid point of the entry edge*/
          arr_mid_x[r] =  (arr_obs_x[r]  + arr_obs_x[r-1])/2;
          arr_mid_y[r] =  (arr_obs_y[r]  + arr_obs_y[r-1])/2;
          ROS_INFO("------obstacle position is postion --%i  --  %f and %f -------", r,arr_obs_x[r], arr_obs_y[r]);
          /*Incrementing the entry edge counter*/
          r++;
          ROS_INFO("R VALUE IS %d",r);
          /*Checking for number of detected edges*/
          if((r >= 2) && (r < 4))
          {
            /*The values less than 4 indicates only one docking edge*/
            ROS_INFO("-------------ONE PLATFORM DETECTED----------------------------");
            /*Storing the mid point of the entry edge*/            
            midxx.push_back(arr_mid_x[1]);           
            midyy.push_back(arr_mid_y[1]);  
            /*Calculation of the average*/ 
            midxxavg = accumulate( midxx.begin(), midxx.end(), 0.0)/midxx.size();   
            midyyavg = accumulate( midyy.begin(), midyy.end(), 0.0)/midyy.size(); 
            final_pos_x = midxxavg;
            final_pos_y = midyyavg;           
            ROS_INFO("------- avg mid points are %f and %f------ ", final_pos_x, final_pos_y);
            ROS_INFO("------- avg diagonal points are %f and %f------ ", diagonal_mid_x, diagonal_mid_y);

          }
          /*This condition states that two entry edges are detected*/
          if(r >= 4)
          {
            /*Accumulating the data*/
            cen_xi_acc.push_back(arr_mid_x[1]);  
            cen_yi_acc.push_back(arr_mid_y[1]); 
            cen_xj_acc.push_back(arr_mid_x[3]);           
            cen_yj_acc.push_back(arr_mid_y[3]);  
            /*Finding the average of mid of entry edge*/
            avgc_xi = accumulate( cen_xi_acc.begin(), cen_xi_acc.end(), 0.0)/cen_xi_acc.size();   
            avgc_yi = accumulate( cen_yi_acc.begin(), cen_yi_acc.end(), 0.0)/cen_yi_acc.size(); 
            avgc_xj = accumulate( cen_xj_acc.begin(), cen_xj_acc.end(), 0.0)/cen_xj_acc.size(); 
            avgc_yj = accumulate( cen_yj_acc.begin(), cen_yj_acc.end(), 0.0)/cen_yj_acc.size();          
            ROS_INFO("-------FINAL CENTRE POINTS ARE %f and %f------ ", (avgc_xi + avgc_xj + diagonal_mid_x) / 3 , (avgc_yi + avgc_yj + diagonal_mid_y) / 3 );                
            ROS_INFO("-------------TWO PLATFORM DETECTED----------------------------");
            /*Calculating the distance of robot from the final detected mid point coordinates*/
            float dist11 = sqrt(pow(avgc_xi - position_data.linear.x , 2) + pow(avgc_yi - position_data.linear.y , 2));
            float dist22 = sqrt(pow(avgc_xj - position_data.linear.x , 2) + pow(avgc_yj - position_data.linear.y , 2));
            /*Checking the closer entry edge out of two detected entry edges*/
            if(dist11 < dist22)
            {
              /*Saving the closer entry edge coordinates*/
              final_pos_x = avgc_xi;
              final_pos_y = avgc_yi;
              /*Saving the farther entry edge coordinates*/
              final_pos_x_far = avgc_xj;
              final_pos_y_far = avgc_yj;
            
            }

            else
            {
              /*Saving the closer entry edge coordinates*/
              final_pos_x = avgc_xj;
              final_pos_y = avgc_yj;
              /*Saving the farther entry edge coordinates*/
              final_pos_x_far = avgc_xi;
              final_pos_y_far = avgc_yi;

            }
            /*Displaying the final calculated mid point coordinates of the entry edge of the trolley for the docking application*/
            ROS_INFO("-------mid points are %f and %f------ ", final_pos_x, final_pos_y);

          } /*end of condition (r >= 4) loop*/

        } /* end of diagonal distance condition check loop*/
      } /* end of condition check loop for all recieved shape of object*/
    
    } /*End of j loop counter*/

  } /*End of i loop counter*/ 

}

int main(int argc, char** argv) 
{
  /*listenernew41 is the name of the node*/
  ros::init(argc, argv, "listenernew41"); 
  /*Creation of the node handle*/
  ros::NodeHandle nh;
  /*ROS Subscriber to retrieve the data from obstacles topic*/
  ros::Subscriber sub = nh.subscribe("obstacles", 1000, counterCallback);
  /*ROS Subscriber to retrieve the data from chatter1 topic*/
  ros::Subscriber sub_lift = nh.subscribe("chatter1", 1000, counterCallback2);
  /*ROS Publishers to publish the data for the motion of the mobile robot*/
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,true);
  ros::Publisher pub1 = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,true);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,true);
  /*ROS Subscriber to retrieve the data from chatter topic*/
  ros::Subscriber sub1 = nh.subscribe("chatter", 1000, counterCallback1);
  /*ROS Publishers to publish the data for the lifting of the platform*/
  ros::Publisher pub_lift1 = nh.advertise<sensor_msgs::Joy>("joy", 1000);
  /*Variables to store the distance of the robot from the detected object*/
  float dis_final = 0;
  float dis_final1 = 0;
  /*Refreshing frequency period*/
  ros::Rate loop_rate(40); 
  /*Creation of ROS TF broadcasters to publish new transformation frames*/
  tf::TransformBroadcaster br1;
  tf::Transform transform1;
  tf::TransformBroadcaster br2;
  tf::Transform transform2;
  tf::TransformBroadcaster br4;
  tf::Transform transform4;
  /*Creation of ROS TF broadcasters to publish new transformation frames*/
  geometry_msgs::PoseStamped motion1;
  geometry_msgs::PoseStamped motion2;
  geometry_msgs::PoseStamped motion3;
  geometry_msgs::PoseStamped motion4;
  geometry_msgs::PoseStamped motion5;
  /*Creation of quaternions to store the euler angles*/
  tf2::Quaternion myQuaternion1;
  tf2::Quaternion myQuaternion2;
  tf2::Quaternion myQuaternion3;
  tf2::Quaternion myQuaternion4;

  float  orientation_z1;
  float  orientation_z2;
  float  orientation_z4;

  /*data for lifting mechanism*/
  vector<int> data1,data2,data3;

  for (int k = 0; k < 11; k++)
  {
    /*Filling the data in the vector, will be used for lifting up  the platform*/
    if((k == 4) || (k==7))
    {
     data1.push_back(1);
    }
    else
    {
     data1.push_back(0);
    }

  }

  for (int k = 0; k < 11; k++)
  {
    /*Filling the data in the vector, will be used for lifting down  the platform*/
    if((k == 4) || (k==6))
    {
      data2.push_back(1);
    }
    else
    {
      data2.push_back(0);
    }

   }

  for (int k = 0; k < 11; k++)
  {
      data3.push_back(0);
  }
    /*datatype to be send on lift publisher topic*/
    sensor_msgs::Joy count2; 


 
   while (ros::ok())
    {
      ros::spinOnce();
      /*Condition to check if the trolley corner is detected*/
      if(((final_pos_x != 0)&&(final_pos_y!= 0)&&(diagonal_mid_x != 0)&&(diagonal_mid_y !=0)) && (!val)){
      ROS_INFO("final mid pts are %f and %f", final_pos_x , final_pos_y);
      /*Calculating the distance of robot from the mid point of the diagonal of the docking edge of trolley*/
      dis_final = sqrt(pow(final_pos_x - position_data.linear.x , 2) + pow(final_pos_y - position_data.linear.y , 2));
      ROS_INFO("DISTANCE IN OUTSIDE LOOP is %f", dis_final);
      /*Checking how far is the robot from the mid point of the diagonal*/
      if(dis_final >= 1.1)
      {
        /*Calculating the robot orientation*/
        orientation_z1 = atan2((diagonal_mid_y - final_pos_y),(diagonal_mid_x - final_pos_x ));
        ROS_INFO("orientation bwetween mid pts and obstacle in degrees is %f", ((orientation_z1 * 180)/M_PI));
        /*Conversion of euler angles to the quaternions to avoid the gimbal lock*/
        myQuaternion1. setRPY( 0,0,orientation_z1);
        myQuaternion1.normalize();
        ROS_INFO("VALUES for tf_broadcaster are %f  and  %f ",final_pos_x,final_pos_y);
        ROS_INFO("VALUES for robot position are %f  and  %f ",position_data.linear.x,position_data.linear.y);
        /*Filling data to the new TF tranform in order to broadcast the new TF*/
        transform1.setOrigin( tf::Vector3(final_pos_x, final_pos_y, 0.0) );
        transform1.setRotation( tf::Quaternion(myQuaternion1.x(), myQuaternion1.y(), myQuaternion1.z(), myQuaternion1.w()) );
        br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "map", "dock_frame1"));
        loop_rate.sleep();
        /*Filling the data for the coordinate frame to be used for the motion execution*/
        motion1.header.frame_id = "/dock_frame1";
        motion1.header.stamp = ros::Time::now();
        motion1.pose.position.x= -0.9;
        motion1.pose.position.y=  0;
        motion1.pose.position.z= 0;
        motion1.pose.orientation.x=  0;
        motion1.pose.orientation.y= 0;
        motion1.pose.orientation.z=  0;
        motion1.pose.orientation.w=  1;
        /*Publishing the data on the move_base_simple/goal topic*/
        pub.publish(motion1);
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();

        ROS_INFO("DISTANCE IN IF LOOP is %f", dis_final);

      } 
      /*Checking how far is the robot from the mid point of the diagonal*/
      if((dis_final >= 0.8) && (dis_final < 1.1))
      {
        ROS_INFO(" ----------------LOOP2----------------");
        /*Calculating the robot orientation*/
        orientation_z1 = atan2((diagonal_mid_y - final_pos_y),(diagonal_mid_x - final_pos_x ));
        /*Conversion of euler angles to the quaternions to avoid the gimbal lock*/
        myQuaternion1. setRPY( 0,0,orientation_z1);
        myQuaternion1.normalize();
        /*Filling data to the new TF tranform in order to broadcast the new TF*/
        transform1.setOrigin( tf::Vector3(final_pos_x, final_pos_y, 0.0) );
        transform1.setRotation( tf::Quaternion(myQuaternion1.x(), myQuaternion1.y(), myQuaternion1.z(), myQuaternion1.w()) );
        br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "map", "dock_frame1"));
        loop_rate.sleep();
        /*Filling the data for the coordinate frame to be used for the motion execution*/
        motion1.header.frame_id = "/dock_frame1";
        motion1.header.stamp = ros::Time::now();
        motion1.pose.position.x= -0.6;
        motion1.pose.position.y=  0;
        motion1.pose.position.z= 0;
        motion1.pose.orientation.x=  0;
        motion1.pose.orientation.y= 0;
        motion1.pose.orientation.z=  0;
        motion1.pose.orientation.w=  1;
        /*Publishing the data on the move_base_simple/goal topic*/
        pub.publish(motion1);
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();

      } 

      else if((dis_final < 0.8) && (r >=4))
      {

          ROS_INFO(" ----------------LOOP3----------------");
          ROS_INFO("GOAL points are %f  and  %f ", 100 * diagonal_mid_x,100 * diagonal_mid_y);
          ROS_INFO("Near Position points are %f  and  %f ",100 * final_pos_x,100 * final_pos_y);
          ROS_INFO("Far Position points are %f  and  %f ",100 * final_pos_x_far,100 * final_pos_y_far);
          /*Calculating the robot orientation*/
          orientation_z2 = atan2((-diagonal_mid_y + final_pos_y_far),(-diagonal_mid_x + final_pos_x_far ));
          ROS_INFO("orientation bwetween mid pts and obstacle in degrees is %f", ((orientation_z2 * 180)/M_PI));
          /*Conversion of euler angles to the quaternions to avoid the gimbal lock*/
          myQuaternion2. setRPY( 0,0,orientation_z2);
          myQuaternion2.normalize();
          ROS_INFO("VALUES for far mid points are %f  and  %f ",final_pos_x_far,final_pos_y_far);
          /*Filling data to the new TF tranform in order to broadcast the new TF*/
          transform2.setOrigin( tf::Vector3(diagonal_mid_x, diagonal_mid_y, 0.0) );
          transform2.setRotation( tf::Quaternion(myQuaternion2.x(), myQuaternion2.y(), myQuaternion2.z(), myQuaternion2.w()) );
          br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "dock_frame2"));
          loop_rate.sleep();
          tf::Matrix3x3 tm1 = transform2.getBasis();
          tf::Vector3 tm2 = transform2.getOrigin();
          tf::Transform trans(tm1, tm2);
          tf::Vector3 translation_check(0.06234 , -0.025, 0);
          tf::Vector3 transformed_vector_check = trans*translation_check;
          ROS_INFO("TRANSFORMED_CHECK VECTOR ARE X = %f and Y = %f",transformed_vector_check.getX() , transformed_vector_check.getY());
          /*Filling the data for the coordinate frame to be used for the motion execution*/
          motion2.header.frame_id = "/dock_frame2";
          motion2.header.stamp = ros::Time::now();
          motion2.pose.position.x=  0.06234; 
          motion2.pose.position.y=  -0.025;
          motion2.pose.position.z= 0;
          motion2.pose.orientation.x=  0;
          motion2.pose.orientation.y= 0;
          motion2.pose.orientation.z= 0;
          motion2.pose.orientation.w=  1;
          /*Publishing the data on the move_base_simple/goal topic*/
          pub1.publish(motion2);
          loop_rate.sleep();
          loop_rate.sleep();
          loop_rate.sleep();
          dis_final2 = sqrt(pow( transformed_vector_check.getX() - position_data.linear.x , 2) + pow( transformed_vector_check.getY() - position_data.linear.y , 2));
          new_val = dis_final2 * 100;
          ROS_INFO("NEW value of dis_final2 is %f", dis_final2 * 100);
          /*Closing the ROS obstacle topic subscriber*/
          sub.shutdown();
          loop_rate.sleep();
          loop_rate.sleep();

        }
        

    }
    else if(val && (new_val33 < old_val33)&& (!val11))
    {

          ROS_INFO("MOVING WITH TROLLEY - ");
          /*Filling the data for the coordinate frame to be used for the motion execution*/
          motion4.header.frame_id = "/map";
          motion4.header.stamp = ros::Time::now();
          motion4.pose.position.x= -0.5;  
          motion4.pose.position.y=  0;
          motion4.pose.position.z= 0;
          motion4.pose.orientation.x=  0;
          motion4.pose.orientation.y= 0;
          motion4.pose.orientation.z= 0;
          motion4.pose.orientation.w=  1;
          //  pub2.publish(motion4); /*CAN BE UNCOMMENTED WHEN PERFORMING MOTION WITH THE TROLLEY CART*/
          loop_rate.sleep();
          loop_rate.sleep();
          loop_rate.sleep();
          loop_rate.sleep();

          ROS_INFO("MOVING WITH TROLLEY --- ");


          ROS_INFO("Robot position is x = %f and y = %f", position_data.linear.x ,position_data.linear.y);
          ROS_INFO("NEW value is %f", new_val33);
          ROS_INFO("OLD value is %f", old_val33);

          old_val33 = new_val33;

    }
    /*Condition for platform to reach a particular point with the trolley*/
    else if((new_val33 >= old_val33) && (!val11))
    {
        ROS_INFO(" ------------------------------REACHED ------------ "); 
        val11 = true;
    }
    /*Desired position reached with the trolley and now the platform can be liftted down*/
    else if(val11)
    {

        ROS_INFO(" ------------------------------LIFTING DOWN  ------------ ");
        count2.header.stamp = ros::Time::now();
        count2.buttons = data2;
        //pub_lift1.publish(count2); /*CAN BE UNCOMMENTED WHEN PERFORMING MOTION WITH THE TROLLEY CART*/
        ros::Duration(0.5).sleep();
        ROS_INFO("LIFT_DOWN");
        ROS_INFO("LIFT POSITION IS %f", position_data_lift.linear.z);
        if((position_data_lift.linear.z > 0)&&(position_data_lift.linear.z < 0.0002))   
        {

        ROS_INFO(" ------------------------------LIFTING DOWN  COMPLETE ------------ ");
        /*Filling the data for the coordinate frame to be used for the motion execution*/
        motion5.header.frame_id = "/map";
        motion5.header.stamp = ros::Time::now();
        motion5.pose.position.x= -1.5;  
        motion5.pose.position.y=  0.01;
        motion5.pose.position.z= 0;
        motion5.pose.orientation.x=  0;
        motion5.pose.orientation.y= 0;
        motion5.pose.orientation.z= 0;
        motion5.pose.orientation.w=  1;
        //pub2.publish(motion5); /*CAN BE UNCOMMENTED WHEN PERFORMING MOTION WITH THE TROLLEY CART*/
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();
        loop_rate.sleep();

      }
    }
  }
    
  return 0; /*End of the main function*/
} 
