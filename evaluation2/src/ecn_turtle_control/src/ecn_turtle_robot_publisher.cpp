/**
\file    ecn_turtle_path_publisher.cpp
\brief  Publishes an rviz arrow marker to represent the robot.

\author(s)  G. Garcia
\date       19 May 2016
*/

//Cpp

#include <math.h>

//ROS
#include "ros/ros.h"

//ROS msgs

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>

/**
  \fn void odomCallback( nav_msgs::Odometry robotPose )
  \brief The callback function which processes robot poses.
  */

bool poseAvailable = false ;
geometry_msgs::Pose2D lastPose ;

void odomCallback( geometry_msgs::Pose2D robotPose ) {
    poseAvailable = true      ;
    lastPose      = robotPose ;
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ecn_turtle_robot_publiser");
    ROS_INFO("ecn_turtle_robot_publiser connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
    ros::NodeHandle nh_global;

    //Subscriptions and publishings
  
    ros::Subscriber poseSub  = nh_.subscribe<geometry_msgs::Pose2D>("pose",1,odomCallback) ;
    ros::Publisher  robotPub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    ros::Rate rate(10);
    while (ros::ok()) {

        ros::spinOnce();

        if( !poseAvailable ){ 
            ROS_INFO("Waiting for robot pose.") ;
            rate.sleep() ;
            continue ;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "ecn_turtle";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW ;            //ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = lastPose.x ;
        marker.pose.position.y = lastPose.y ;
        marker.pose.position.z = 0.0 ;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(lastPose.theta);

        marker.pose.orientation = quat ;

        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        robotPub.publish( marker );

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
