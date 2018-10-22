
/*
 * main.cpp
 *
 *  Created on: 2017-11-9
 *      Author: ophelia
 */

#include <ros/ros.h>
#include <nxt_control/SensorData.h>

void count(const nxt_control::SensorData msg){
	static int countA=0;
	static int countB=0;
	countA+=msg.tickCountA;
	countB+=msg.tickCountB;
	double dist = msg.distance;
	ROS_INFO("%d,%d,%f",countA,countB, dist);
	//std::cout<<countA<<','<<countB<<std::endl;

}
int main(int argc, char** argv){
	ros::init(argc, argv, "brain");
	ros::NodeHandle n;
	ros::Subscriber sub;
	sub=n.subscribe("sensor data", 1,count);
	while(ros::ok())
	{
		ros::spinOnce();
	}
	//ros::spin();
	return 1;
}



