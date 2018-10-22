/*
 * Scananalyzer.h
 *
 *  Created on: Feb 22, 2018
 *      Author: ophelia
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <list>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <opencv2/core.hpp>
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#ifndef SCANANALYZER_H_
#define SCANANALYZER_H_

class Scan_analyzer {
public:
	// datatypes //
	struct safeSector
	{
		bool SAS;
		int SAS_idx, SAE_idx; //Safe Angle Start Point, Safe Angle End Point
		double SAS_dist, SAE_dist;
		//depth of Safe Angle Start point
		//depth of Safe Angle End point
	};
	double radius; //threshold of safe distance

	// pose / twist
	geometry_msgs::Pose2D robotPose;
	geometry_msgs::Pose2D lastPose;
	geometry_msgs::Twist drive_command;
	tf::Transform scanPose;

	bool init;
	double scan_min;
	bool isSafe_r;
	bool isSafe_f;
	bool isSafe_l;
	double angle_increment;
	double angle_min;
	double angle_max;
	double range_min;
	double range_max;
	double angle_offset;
	double D_safe; //Safe width that the robot can pass through
	std::vector<float> actScan;
	std::vector<float> actScan_last;
	std::vector<safeSector> sector_list;
	cv::Mat map;
	int old_x,old_y;
	int new_x, new_y;

	void refresh(const sensor_msgs::LaserScan &,geometry_msgs::Pose2D);
	std::vector<safeSector> getSectors(double);
	bool sector_IsSafe(int, double);
	double getMinDist();
	std::vector<double> getSafeAngles(double radius);//return all safe angles
	std::vector<float> smoothScan(std::vector<float>);
	void updateMap();
	void updateMapPose();


	Scan_analyzer();
	virtual ~Scan_analyzer();
};

#endif /* SCANANALYZER_H_ */
