/**
 * \file dead_simple_platoon_node.cpp
 * \brief The platoon leader
 * \author Gaetan Garcia 
 * \version 0.1
 * \date 14 April 2018
 * 
 * \param[in] 
 *    ° radius: radius of the circle along which the leader moves.
 *      Motion starts at posture (radius,0,pi/2).
 * 
 * Subscribes to: <BR>
 *    ° "/key", absolute topic name, type std_msgs::Int16,
 *         which indicates when to accelerate/decelerate. <BR>
 * 
 * Publishes to: <BR>
 *    ° "/leader_pos", absolute topic name, type geometry_msgs::Pos,
 *         position of the virtual leader. <BR>
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

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// Accel/decel default keys: '+' and '-'
#define DEFAULT_ACCEL_KEY 43
#define DEFAULT_DECEL_KEY 45

#define DEFAULT_RADIUS 5.0
#define DEFAULT_COLOR "blue"

int accelKey, decelKey ;
double vel = 3.0 ;

// Callback functions to handle keystroke messages.

void keystrokeCallback(std_msgs::Int16 key_msg){
    if( key_msg.data == accelKey ){
        if( vel < 10.0 ) vel += 1.0 ;
        ROS_INFO("Velocity: %f",vel) ;
    }else if( key_msg.data == decelKey ){ // Decelerate, unless already very slow.
        if( vel > 2.0 )  vel -= 1.0 ;
        ROS_INFO("Velocity: %f",vel) ;
    }
}

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;


// Our marker message has many fields that remain constant and
// are initialized only once by the following function.

void initializeMarker(){
    // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
    std::string nodeName ;
    nodeName = ros::this_node::getName();
    // Create a marker for this node. Only timestamp and position will be later updated.
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
}


// Function to publish a marke at a given (x,y) position.

void publishMarkerAt( geometry_msgs::Point markerPos) {    
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.x ;
    marker.pose.position.y = markerPos.y ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "eval_1_gr_a_node");
    
// ------------- 
// Your modifications should be within the two commented dashed lines.    

    // Node handles
    ros::NodeHandle nh, nh_loc("~") ;

    // Read the node parameters.
    double radius ;
    nh_loc.param("accel_key"  , accelKey , DEFAULT_ACCEL_KEY);
    ROS_INFO("accel_key: %d",accelKey) ;
    nh_loc.param("decel_key"  , decelKey , DEFAULT_DECEL_KEY);
    ROS_INFO("decel_key: %d",decelKey) ;
    nh.param("radius"  , radius , DEFAULT_RADIUS);
    
    std::string color ;
    nh.param<std::string>("color",color,DEFAULT_COLOR) ;
    
    // Topics to which the node subscribes.
    ros::Subscriber subKeystrokes = 
        nh.subscribe<std_msgs::Int16>("the_key_typed",1,keystrokeCallback);

    // Topics to which the node publishes. The position of the leader is given
    // by its polar angle on the circle, hence the name of the topic.
    ros::Publisher pubPolarAngle = nh.advertise<std_msgs::Float32>("leader_angle",1);
    
// Your modifications should not go beyond this line.
// -------------
    
    pubMarker = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    geometry_msgs::Point leader ;
    leader.x = radius ;
    leader.y = 0.0 ;
    leader.z = 0.0 ;
    double rho = 0.0 ;   // Polar angle of leader along circle.
    
    ros::Time t, prev_t ;
    prev_t = ros::Time::now() ;
    
    if(color=="red"){
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
    }else if( color=="green" ){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
    }if( color=="blue" ){
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5f;
    }
    
    initializeMarker() ;

    ros::Rate rate(20);
    while (ros::ok()){

        ros::spinOnce();

        // The leader moves at constant speed along the 
        // circular trajectory.
        t = ros::Time::now() ;
        ros::Duration d ;
        d = t-prev_t ;
        rho += (vel/radius)*d.toSec() ;
        leader.x = radius*cos(rho) ;
        leader.y = radius*sin(rho) ;
        publishMarkerAt(leader) ;
        prev_t = t ;
        
        std_msgs::Float32 polarAngleMessage ;
        polarAngleMessage.data = rho ;
        pubPolarAngle.publish(polarAngleMessage) ;

        rate.sleep();
    }
}
