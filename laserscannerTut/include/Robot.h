/*
 * Robot.h
 *
 *  Created on: Oct 27, 2017
 *      Author: adminuser
 */

#ifndef ROBOT_H_
#define ROBOT_H_
#include<vector>
#include "Sensor.h"
#include "Parkour.h"
#include <opencv2/opencv.hpp>
class Robot {
public:
	Pose pose;
	void measurement(int i, const Parkour p,std::vector<double>& measurement);
	std::vector<Sensor> sensors;
	cv::Mat shape;
	cv::Scalar transparentColor;
	Robot();
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
