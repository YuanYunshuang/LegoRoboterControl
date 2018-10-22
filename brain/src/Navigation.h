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
#include "brain/MotorCommand.h"
#include "brain/SensorData.h"
#include "sensor_msgs/Range.h"
#include <opencv2/core.hpp>

#include "Scananalyzer.h"

#ifndef SRC_Navigation_H_
#define SRC_Navigation_H_

class Navigation {
public:
public:

	// datatypes //

		struct goal
		{
			double x,y;
		};
		// Dijkstra node element, which describes a another point linked to itself
		// and the distance to this linked point
		struct RTcell //route table cell
		{
			int id; //cell id
			double cost;
			int pred_id; //id of predecessor
			bool marked; //optimal predecessor and cost found-->set to true
		};

		struct vertex
		{
			goal coor;
			int id;
		};

		struct lane
		{
			std::vector<goal> points;
			goal pstart;
			goal pend;
			double length;
			std::vector<goal> mid_line;
			bool noObj;
		};
		typedef std::vector<RTcell> RT; //RT:Route table for a single vertex

		Navigation::goal localGoal;
		Navigation::goal Goal;

		// pose / twist
		geometry_msgs::Pose2D robotPose;
		geometry_msgs::Pose2D lastPose;
		geometry_msgs::Twist drive_command;
		tf::Transform scanPose;
		Scan_analyzer scan_analyzer;


		//tf::Transform laser_transform;
		bool init;
		bool ultraPass;
		bool routeOn;
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
		std::vector<goal> route;
		std::vector<vertex> vertex_list;
		cv::Mat Dij_map;
		cv::Mat table_links;
		RT GlobalRT;


		// NodeHandle
		ros::NodeHandle n_;
		//ros::Time tracking_pose_delay;

		// ros subscribers
		ros::Subscriber sub_scan;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_ultra;

		// ros punblishers

		ros::Publisher drive_pub;
		ros::Publisher rotate_pub;
		ros::Publisher effort_pub;
		// Methods
		void ultraCallback(const brain::SensorData);
		void robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped &);
		void scannerCallback(const sensor_msgs::LaserScan &);
		brain::MotorCommand pubStop();
		Navigation::goal getLocalGoal();
		std::vector<Navigation::goal> getRoute();
		std::vector<Navigation::goal> getShortestPath(vertex, vertex);
		Navigation::vertex matchVertex(goal);
		Navigation::RT getRouteTablefor(vertex);
		RTcell findMinCell(RT&);
		bool allVertexMarked(RT);
		lane getLane(goal, goal);
		void refreshLinksTable();
		void refreshDijVertexList();
		void drive2Goal(Navigation::goal);
		bool goal_reached(Navigation::goal);
		void rotate2AimAngle(double);
		void rotate(double );
		void drive(double );


	Navigation(ros::NodeHandle , Scan_analyzer);
	virtual ~Navigation();
};

#endif /* SRC_Navigation_H_ */
