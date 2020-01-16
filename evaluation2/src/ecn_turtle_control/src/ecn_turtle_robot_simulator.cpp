/**
\file    ecn_turtle_robot_simulator.cpp
\brief  Kinematic simulation of a differential drive robot
 *
 *  Not much to say, just an exercise...
 *
\author(s)  G. Garcia
\date       19 May 2016
*/

//Cpp

#include <math.h>

//ROS
#include "ros/ros.h"

//ROS msgs

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/Pose2D.h> 

double    circleRadius, x, y, theta ;
bool      velocityReceived = false ;
ros::Time lastUpdate ;
unsigned  seqNumber = 0 ;

ros::Publisher posePub ;

void velocityCallback( geometry_msgs::Vector3 velocityCommand ){

    if( !velocityReceived ) {

        lastUpdate = ros::Time::now() ;
        velocityReceived = true ;

    }else{

        ros::Time currentTime = ros::Time::now() ;

        ros::Duration deltaT = currentTime-lastUpdate ;

        x     += velocityCommand.x * deltaT.toSec() * cos(theta) ;
        y     += velocityCommand.x * deltaT.toSec() * sin(theta) ;
        theta += velocityCommand.z * deltaT.toSec()             ;

        geometry_msgs::Pose2D newPose ;

        newPose.x     = x     ;
        newPose.y     = y     ;
        newPose.theta = theta ;

        lastUpdate = currentTime ;  

        posePub.publish(newPose) ; 

    }
}


void radiusCallback( std_msgs::Float32 radiusMessage ){
    circleRadius = radiusMessage.data ;
}


// Normalize angle to interval [ -pi , +pi ]

double normAngle( double angle ) {
    while( angle >  M_PI ) angle -= 2.0*M_PI ;
    while( angle < -M_PI ) angle += 2.0*M_PI ;
    return angle ;
}



int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ecn_turtle_control_node");
    ROS_INFO("ecn_turtle_control_node connected to roscore");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
    ros::NodeHandle nh_global;

    // Retrieving global parameter, which define the circular path (centered at 0,0) 
    // and the robot's initial posture.

    double x0,y0,theta0 ;

    if( !nh_.getParam("/circleRadius",circleRadius) ){
        ROS_FATAL("Couldn't find parameter: circleRadius\n") ;
        return 1 ;
    }else{
        ROS_INFO("Circle radius: %lf",circleRadius) ;
    }
    if( !nh_.getParam("/x0",x0) ){
        ROS_FATAL("Couldn't find parameter: x0\n") ;
        return 1 ;
    }else{
        ROS_INFO("x0: %lf",x0) ;
    }
    if( !nh_.getParam("/y0",y0) ){
        ROS_FATAL("Couldn't find parameter: y0\n") ;
        return 1 ;
    }else{
        ROS_INFO("y0: %lf",y0) ;
    }
    if( !nh_.getParam("/theta0",theta0) ){
        ROS_FATAL("Couldn't find parameter: theta0\n") ;
        return 1 ;
    }else{
        ROS_INFO("theta0: %lf",theta0) ;
    }

    //Subscribe to the velocity command and publish the 2D pose of the robot.
  
    ros::Subscriber velSub    = nh_.subscribe<geometry_msgs::Vector3>("/desiredVelocity",1,velocityCallback) ;
    ros::Subscriber radiusSub = nh_.subscribe<std_msgs::Float32>("/circleRadius",1,radiusCallback) ;
    ros::Publisher  errorPub  = nh_.advertise<geometry_msgs::Point>("/controlError",1) ;
                    posePub   = nh_global.advertise<geometry_msgs::Pose2D>("pose",1) ;

    x = x0 ;   y = y0 ;   theta = theta0 ;
    geometry_msgs::Point poseError;

    ros::Rate rate(50);
    while (ros::ok()) {

        ros::spinOnce();
        
        double polarAngle = atan2( y , x ) ;
        double refHeading = polarAngle + M_PI/2.0 ;
        double headingError = normAngle( refHeading - theta ) ;
        double lateralError = sqrt( x*x + y*y ) - circleRadius ;

        poseError.x = 0.0          ;
        poseError.y = lateralError ;
        poseError.z = headingError ;

        errorPub.publish(poseError) ;

        rate.sleep();

    }

    ROS_INFO("ROS-Node Terminated\n");
}
