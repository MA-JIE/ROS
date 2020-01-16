/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic types you use.
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h> 

// You may have a number of globals here.

double circleRadius;
double thetae,ye;
double Ky, Ktheta, Vref;
bool errorAvailable = false;
geometry_msgs::Vector3 velCommand;
// Callback functions...

void errorCallback(geometry_msgs::Point errorMessage){
	thetae = errorMessage.z;
	ye = errorMessage.y;
}

void radiusCallback(std_msgs::Float32 radiusMessage){
    circleRadius = radiusMessage.data;
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "ecn_turtle_control_node");
    ROS_INFO("ecn_turtle_control_node connected to roscore");

    // Define your node handles
    ros::NodeHandle nh_("~");

    // Read the node parameters if any

    if( !nh_.getParam("/Ktheta",Ktheta) ){
        ROS_FATAL("Couldn't find parameter: Ktheta\n") ;
        return 1 ;
    }else{
        ROS_INFO("Ktheta: %lf",Ktheta) ;
    }
    if( !nh_.getParam("/Ky",Ky) ){
        ROS_FATAL("Couldn't find parameter: Ky\n") ;
        return 1 ;
    }else{
        ROS_INFO("Ky: %lf",Ky) ;
    }
    if( !nh_.getParam("/Vref",Vref) ){
        ROS_FATAL("Couldn't find parameter: Vref\n") ;
        return 1 ;
    }else{
        ROS_INFO("Vref: %lf",Vref) ;
    }

    if( !nh_.getParam("/circleRadius",circleRadius) ){
        ROS_FATAL("Couldn't find parameter: circleRadius\n") ;
        return 1 ;
    }
    // Declare your node's subscriptions and service clients
    ros::Subscriber errorSub =nh_.subscribe<geometry_msgs::Point>("/controlError",1,errorCallback) ;
    ros::Subscriber radiusSub = nh_.subscribe<std_msgs::Float32>("/circleRadius",1,radiusCallback);
    
    // Declare you publishers and service servers
    ros::Publisher velPub = nh_.advertise<geometry_msgs::Vector3>("/desiredVelocity",1);
    

    ros::Rate rate(50);   // Or other rate.
	while (ros::ok()){
		ros::spinOnce();

		// Your node's code goes here.

		velCommand.x = Vref;

		velCommand.z = Vref/circleRadius+ Ky*Vref*ye + Ktheta*thetae;
                velPub.publish(velCommand);
        

		rate.sleep();
    }
    ROS_INFO("ROS-Node Terminated\n");
}


