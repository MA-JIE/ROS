/**
 * \file vehicle_gr_a.cpp
 * \brief The node implements the behavior of a vehicle in the platoon.
 * \author Gaetan Garcia 
 * \version 0.1
 * \date 14 April 2018
 * 
 * \param[in] index, private parameter. Positive integer. 
 *      Position of the vehicle in the platoon. 
 * 
 * Subscribes to: <BR>
 *    ° "virtual_leader_pos", group topic name, type geometry_msgs::Pos,
 *         position of the virtual leader. <BR>
 * 
 * Publishes to: <BR>
 *    ° "pos", local topic name, type geometry_msgs::Point, position
 *         of this swarm member. <BR>
 *    ° "/visualization_marker", absolute topic name, visualization_msgs::Marker,
 *         to publish to rviz a marker at the position of the swarm member. <BR>
 *
 * The behavior of the swarm member is to get to a random point close 
 * to the virtual leader.
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <std_msgs/Float32.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// You may have a number of globals here.

#define DEFAULT_RADIUS 5.0
#define DEFAULT_COLOR "blue"

// Callback functions...

bool leaderInfoReceived = false ;
double desiredRho ;
int rank ;
double radius ;
std::string color ;

void leaderPolarAngleCallback(std_msgs::Float32 leader_polar_angle_msg){
    // Copy leader information into global variable
    desiredRho = leader_polar_angle_msg.data -rank * 15.0*M_PI/180 ;
    leaderInfoReceived = true ;
}

ros::Publisher pubMarker ;
visualization_msgs::Marker marker;

void initializeMarker( ){
    // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
    std::string nodeName ;
    nodeName = ros::this_node::getName();
    // Create a marker for this node. Only timestamp and position will be later updated.
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
}

void publishMarkerAt( geometry_msgs::Point markerPos) {    
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.x ;
    marker.pose.position.y = markerPos.y ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}

#define FREQ 10.0

int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "vehicle_gr_a");
   
// ------------- 
// Your modifications should be within the two commented dashed lines.

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob ;

    nh_loc.param("rank",rank,1); 
    nh_glob.param("radius",radius,DEFAULT_RADIUS) ;  
    nh_glob.param<std::string>("color",color,DEFAULT_COLOR) ;
    
    // What the node subscribes to. The position of the leader is given
    // by its polar angle on the circle, hence the name of the topic.
    ros::Subscriber subToPolarAngle = 
        nh_glob.subscribe<std_msgs::Float32>("leader_angle",1,leaderPolarAngleCallback);
  
// Your modifications should not go beyond this line.
// -------------

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
    
    pubMarker = nh_loc.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;

    // Initial position.
    double rho = - rank * 15.0*M_PI/180.0 ;
    geometry_msgs::Point pos ;
    pos.x = radius * cos(rho) ;
    pos.y = radius * sin(rho) ;
    pos.z = 0.0 ;
    publishMarkerAt( pos ) ; 
    
    initializeMarker() ;
        
    ros::Rate rate(FREQ);
    while (ros::ok()){

        ros::spinOnce();
        
        // Just set the position of the follower at the desired position.
        // No control, nothing... Not a true platoon control, of course.
        if( leaderInfoReceived ){
            pos.x = radius * cos(desiredRho) ;
            pos.y = radius * sin(desiredRho) ;
            pos.z = 0.0 ;
            publishMarkerAt( pos ) ; 
        }
            
        rate.sleep();
    }
}
