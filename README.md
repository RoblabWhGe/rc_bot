rc_bot
======

A ROS Repository for the radio controlled disposable robot develeb by the Robotics Lab in the Computer Science Department of the University of Applied Science Gelsenkirchen

## Clone our repo to obtain all packages 
> git clone https://github.com/RoblabWhGe/rc_bot

## Low Budget RC Robot using Raspberry PI and ROS
### Summary
The RC Bot is a low budget robot based and a RC Car and a RaspberryPI running ROS Hydro.
Most RC Cars can be controlled via several pwm signals e.g. one for the driving motor and one for the steering servo. This Package receives Twist messages and converts them to corresponding pwm signals. Also a simple webcam is mounted on the robot, so that we can receive a live camera stream.
![RC BOT"](http://s1.directupload.net/images/131206/n4lqviou.jpg "RC BOT")
### Installation and usage
- Install ROS Fuerte on RaspberryPI and create a workspace. You can follow the official tutorials if you have further questions.
- For controlling the GPIO on RPi wiringPi and PIBlaster are needed

> https://projects.drogon.net/raspberry-pi/wiringpi/download-and-install/

> https://github.com/sarfata/pi-blaster/
syn 
- Copy neccesary nodes, rc_bot and rc_bot_bringup to your Workspace and compile them using catkin_make
- Call roslauch rc_bot_bringup rc_bot.launch on the RaspberryPI to start the robot
- You can now send Twist message and receive the compressed camera stream.
- You can also setup another ROS machine to control the robot over network. 

