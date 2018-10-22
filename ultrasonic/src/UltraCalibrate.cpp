/*
 * UltraCalibrate.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: ophelia
 */

#include "UltraCalibrate.h"

UltraCalibrate::UltraCalibrate(ros::NodeHandle n) {
	// TODO Auto-generated constructor stub
	subSensors = _n.subscribe("sensor_data", 10, &Control::getSensorData, this);
}

UltraCalibrate::~UltraCalibrate() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv){
	ros::init(argc, argv, "brain");
		ros::NodeHandle node;
		UltraCalibrate x = TickCounter(node);
		ros::spin();				//check for incoming msg

		return 0;
}
