/*
 * Navigation.h
 *
 *  Created on: Jan 17, 2018
 *      Author: ophelia
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <list>
#include "sensor_msgs/LaserScan.h"
#include "brain2/MotorCommand.h"
#include "brain2/SensorData.h"

#ifndef SRC_Navigation_H_
#define SRC_Navigation_H_

class Navigation {
public:
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

		struct goal
		{
			double x,y;
		};
		Navigation::goal End;
		Navigation::goal localGoal;
		Navigation::goal Goal;
		Navigation::goal dangerGoal;
		double M; //threshold of safe distance
		double R; //threshold of step length

		// pose / twist
		geometry_msgs::Pose2D robotPose;
		geometry_msgs::Pose2D lastPose;
		geometry_msgs::Twist drive_command;
		tf::Transform scanPose;


		//tf::Transform laser_transform;
		bool init;
		bool next_cmd;
		double angle_increment;
		double angle_min;
		double angle_max;
		double range_min;
		double range_max;
		double angle_offset;
		double D_safe; //Safe width that the robot can pass through
		double last_direct;
		double current_direct;
		double cost1, cost2, cost3;
		double turn_threshold;
		double turn_step;
		double drive_tol;
		double drive_dist;
		double rotate_angle;
		std::vector<float> actScan;
		std::vector<float> actScan_last;
		std::vector<safeSector> sector_list;
		//std::vector<vector<float>> ChoiceAngles;

		// NodeHandle
		ros::NodeHandle n_;
		ros::Time tracking_pose_delay;

		// ros subscribers
		ros::Subscriber sub_scan;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_nextcmd;

		// ros punblishers

		ros::Publisher drive_pub;
		ros::Publisher rotate_pub;
		ros::Publisher effort_pub;
		// Methods
		void nextcmdCallback(const std_msgs::Bool);
		void scannerCallback(const sensor_msgs::LaserScan &);
		Navigation::goal getLocalGoal(const std::vector<Navigation::safeSector>);
		std::vector<Navigation::safeSector> getSectors(const std::vector<float>);
		void robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped &);
		//void robotInitPosCallback(const geometry_msgs::PoseWithCovarianceStamped &robInitPos)
		brain2::MotorCommand pubStop();
		void drive2Goal(Navigation::goal);
		bool goal_reached(Navigation::goal);
		void rotate(double );
		void drive(double );


	Navigation(ros::NodeHandle n);
	virtual ~Navigation();
};

#endif /* SRC_Navigation_H_ */
