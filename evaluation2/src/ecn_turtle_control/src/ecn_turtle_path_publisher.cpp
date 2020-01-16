/**
\file   ecn_turtle_path_publisher.cpp
\brief  Publishes the desired path of the turtlebot so it
        can be visualized under rviz.

\author(s)  G. Garcia
\date       19 May 2016
*/

//Cpp

#include <math.h>

//ROS
#include "ros/ros.h"

//ROS msgs

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

double circleRadius ;
//int incre, decre;

ros::Publisher radiusPub ;


// On receiving a valid code, kbdCallback modifies the
// radius of the circular trajectory and publishes it.

void kbdCallback( std_msgs::Int16 key_msg ) {
    std_msgs::Float32 radiusMessage ;
    if( key_msg.data == 43 ){
        if( circleRadius < 4.0 ) circleRadius += 0.5 ;
        radiusMessage.data = circleRadius ;
        radiusPub.publish(radiusMessage) ;
        ROS_INFO("Radius: %lf",circleRadius);
    }
    if( key_msg.data == 45){
        if( circleRadius > 1.0 ) circleRadius -= 0.5 ;
        ROS_INFO("Radius: %lf",circleRadius);
        radiusMessage.data = circleRadius ;
        radiusPub.publish(radiusMessage) ;
    }
}


int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ecn_turtle_path_publiser");
    ROS_INFO("ecn_turtle_path_publiser connected to roscore");
    ros::NodeHandle nh_("~");  //ROS Handler - local namespace.

    // Retrieving parameters. The path is a circle centered at (0,0)

    if( !nh_.getParam("/circleRadius",circleRadius) ){
        ROS_FATAL("Couldn't find parameter: circleRadius\n") ;
        return 1 ;
    }

    //Subscriptions and publishings

    ros::Subscriber kbdSub    = nh_.subscribe<std_msgs::Int16>("/key_typed",1,kbdCallback) ;
    ros::Publisher  pathPub   = nh_.advertise<nav_msgs::Path>("/desired_path",1) ;
                    radiusPub = nh_.advertise<std_msgs::Float32>("/circleRadius",1) ;

    double angleRes = 5*M_PI/180.0 ;  // 5°
    unsigned seqNumber = 0 ;

    geometry_msgs::Quaternion id ;
    id.x = 0.0 ;
    id.y = 0.0 ;
    id.z = 0.0 ;
    id.w = 1.0 ;

    ros::Rate rate(1);
    while (ros::ok()) {

        ros::spinOnce();

        nav_msgs::Path path ;

        path.header.frame_id = "/map"           ;
        path.header.stamp    = ros::Time::now() ;
        path.header.seq++ ;

        // Defining the circular path.
        for( unsigned i = 0 ; i <= 2*M_PI/angleRes ; i++ ) {
            geometry_msgs::PoseStamped ps ;
            double angle = i*angleRes ;
            ps.header = path.header ;
            ps.pose.position.x  = circleRadius*cos(angle) ;
            ps.pose.position.y  = circleRadius*sin(angle) ;
            ps.pose.position.z  = 0.0                     ;
            ps.pose.orientation = id                      ;
            path.poses.push_back(ps)                      ;
        }

        pathPub.publish(path) ;

        rate.sleep();

    }

    ROS_INFO("ROS-Node Terminated\n");
}
