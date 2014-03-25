/**
 * Copyright (c) 2012, University of Applied Sciences Gelsenkirchen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Washington University in St. Louis nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * File: joy_controller.cpp
 * Author: Stefan Wilkes
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

/**
 * This class lets you control a robot over a joystick by sending
 * standard twist commands.
 */
class JoyController
{

private:

	/** ROS node handle */
	ros::NodeHandle n;

	/** Subscrier for joysticks messages */
	ros::Subscriber JoyNode_sub;

	/** Publisher for roombas speed msgs */
	ros::Publisher robotSpeedPub;
	
	/** Max linear speed for the robot */
	double maxLinearVel;
	
	/** Max angular speed for the robot */
	double maxAngularVel;

	/**
	 * Subscriber callback to process joystick messages and publish
	 * velocity commands.
	 */
	void joyCallback(const sensor_msgs::JoyConstPtr &joystick);

public:

    /**
     * constructor of JoyController
     *
     * set enable to true
     * initialize ServiceServer and Subscriber
     */
	JoyController();
	
	/**
	 * Handles the node logic
	 */
	void mainNodeLoop(void);
};

JoyController::JoyController()
{
    /* Get private parameters */
	ros::NodeHandle priavteHandle("~");
	priavteHandle.param<double>("max_linear_vel", this->maxLinearVel, 1);
	priavteHandle.param<double>("max_angular_vel", this->maxAngularVel, 1);

	JoyNode_sub = n.subscribe("joy", 10, &JoyController::joyCallback, this);
	this->robotSpeedPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void JoyController::mainNodeLoop(void)
{
    /* Continously process messages */
	ros::spin();
}

void JoyController::joyCallback(const sensor_msgs::JoyConstPtr &joystick)
{
	geometry_msgs::Twist robotSpeed;
	robotSpeed.linear.x = joystick->axes[1] * this->maxLinearVel;
	robotSpeed.linear.y = 0;
	robotSpeed.linear.z = 0;
	robotSpeed.angular.x = 0;
	robotSpeed.angular.y = 0;
	robotSpeed.angular.z = joystick->axes[0] * this->maxAngularVel;

	this->robotSpeedPub.publish(robotSpeed);
}

/**
 * Main function:
 * Creates a new joy controller, initilizes the ros environment and starts 
 * processing.
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_teleop_joy");
	
	JoyController joyController;
	joyController.mainNodeLoop();
}
