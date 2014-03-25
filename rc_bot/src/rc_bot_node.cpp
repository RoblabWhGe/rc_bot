/*
 * This file is part of the rc_bot package.
 *
 * Copyright (c)2014 by Robotics Lab 
 * in the Computer Science Department of the 
 * University of Applied Science Gelsenkirchen
 * 
 * Author: Stefan Wilkes
 *  
 * The package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <wiringPi.h>

#define PWM_CONTROL_FILE "/dev/pi-blaster"
#define SERVO_PWM_CHANNEL 5
#define MOTOR_PWM_CHANNEL 4

using namespace std;

/**
 * A one line description of this node.
 *
 * A longer description of this node.
 *
 * @author Stefan Wilkes
 */ 
class RCBot 
{
 public:
	/**
	 * Constructor
	 */
	RCBot();

	/**
	 * Destructor
	 */
	~RCBot();

	/**
	 * Main node loop.
	 */
	void mainNodeLoop();

private:

	/** The node handle **/
	ros::NodeHandle nodeHandle;

	/** A publisher member **/
	// ros::Publisher mPub;

	/** A Subscriber member **/
	ros::Subscriber velocitySub;

	/** A Service Server member **/
	//ros::ServiceServer mServServer;

	FILE *pwmControlFile;

	bool driveForwards;

	/**
	 * Subscriber callback to the velocity topic.
	 *
	 * @param vel The new velocity for the Bot
	 */
	void velocitySubCallback(const geometry_msgs::Twist::ConstPtr &vel);

	/**
	 * The FooBar Service callback.
	 *
	 * @param req
	 * @param resp
	 * @return
	 */
	// bool FooBar_Service(ClassName::FooBar::Request &req, ClassName::FooBar::Response &resp)
};

RCBot::RCBot()
{
	/* Register subscriber and services */
	this->velocitySub = this->nodeHandle.subscribe("rc_bot/cmd_vel", 1, &RCBot::velocitySubCallback, this);

	/* Open files for hardware communication */
	this->pwmControlFile = fopen(PWM_CONTROL_FILE, "w");
	
	if (this->pwmControlFile == NULL)
	{
 		ROS_ERROR("Can't open hardware file: %s", PWM_CONTROL_FILE);
	}

	/* Setup WiringPi 11 (GPIO7) for toggeling direction pin */
	system("gpio mode 11 out");
	system("gpio write 11 1");
	this->driveForwards = true;
}

RCBot::~RCBot()
{
	/* Close previosly opened files */
	fprintf(this->pwmControlFile, "%i=0\n", MOTOR_PWM_CHANNEL);
	fflush(this->pwmControlFile);
	fprintf(this->pwmControlFile, "%i=0\n", SERVO_PWM_CHANNEL);
	fflush(this->pwmControlFile);
	fclose(this->pwmControlFile);
}

void RCBot::mainNodeLoop()
{
	/* ******************************************
	 * Sleep till new messages arrives
	 * ******************************************/
	ros::spin();
}

void RCBot::velocitySubCallback(const geometry_msgs::Twist::ConstPtr &vel)
{
	/* Check if direction is set correctly */
	if (vel->linear.x > 0 && !driveForwards)
	{
		system("gpio write 11 1");
		this->driveForwards = true;	
	}
	else if (vel->linear.x < 0 && driveForwards)
	{
		system("gpio write 11 0");
		this->driveForwards = false;
	}

	/* Set angular speed to servo (PWM) */
	/* PWM 0.075 DC is the center pose. Left / Right = +/- 0.02 */
	double offset = vel->angular.z * 0.025;
	offset = (offset > 0.025) ? 0.025 : (offset < -0.025) ? -0.025 : offset;
	fprintf(this->pwmControlFile, "%i=%f\n", SERVO_PWM_CHANNEL, offset + 0.072);
	fflush(this->pwmControlFile);


	/* Set linear speed to motor (PWM) 0% ... 100% */
	double speed = abs(vel->linear.x);
	speed = (speed > 1) ? 1 : speed;
	fprintf(this->pwmControlFile, "%i=%f\n", MOTOR_PWM_CHANNEL, speed);
	fflush(this->pwmControlFile);

	//ROS_INFO("New PWM Duty Settings (Servo | Motor): %f | %f", offset + 0.075, speed);
}

/**
 * Main Method
 *
 * @param argc Number of Arguments given to this method
 * @param argv Pointer to the list of arguments (char streams)
 * @return zero if method finished successfully
 */ 
int main(int argc, char** argv)
 {
	// Register node in the ros environment
	ros::init(argc, argv, "rc_bot_node");
	RCBot instance;
	instance.mainNodeLoop();
}
