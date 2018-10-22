/*
 * Navigation.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: ophelia
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <iostream>
#include <list>
#include <limits>
#include "sensor_msgs/PointCloud2.h"
#include <math.h>
#include <laser_geometry/laser_geometry.h>
#include "Navigation.h"
#include <stdlib.h>
// Abfragen
// bool dr2go;
bool rot_done = true;
bool drv_done = true;
bool change_direct = false;
bool thr_small = true;
bool dangerGoalOn = false;
bool driveOn = true;
ros::Time time_start;
ros::Time time_rotl;
ros::Time time_rotr;
ros::Time time_pose;

Navigation::Navigation(ros::NodeHandle n) {
	// TODO Auto-generated constructor stub
	n_ = n;
	time_start = ros::Time::now();
	time_rotl = ros::Time::now();
	time_rotr = ros::Time::now();
	time_pose = ros::Time::now();
	//next_cmd = false;
	angle_increment = 0;
	angle_min = 0;
	D_safe = 0.25;
	last_direct = 0.0;
	current_direct = 0.0;
	cost1 = 7.0;
	cost2 = 10.0;
	cost3 = 2.0;
	turn_threshold = 0.15;
	turn_step = 0.3;
	drive_tol = 0.1;
	angle_offset = 20;
	R = 0.4;
	M = 0.4;
	//Navigation::goal Goal;
	std::cin >> Goal.x;
	std::cin >> Goal.y;

	localGoal.x = 0.0;
	localGoal.y = 0.0;
	dangerGoal.x = 0.0;
	dangerGoal.y = 0.0;
	robotPose.x = 0.0;
	robotPose.y = 0.0;
	robotPose.theta = 0.0;
	lastPose.x = 0.0;
	lastPose.y = 0.0;
	lastPose.theta = 0.0;
	//scanPose.setOrigin (tf::Vector3( (0.0), (0.0), (0.0) )  ); // pose is 0
	//scanPose.setRotation (tf::Quaternion ((0.0), (0.0), (0.0), (1.0) )  );
	init = false; //makiert ob die Roboterposition initialisiert wurde

	tracking_pose_delay = ros::Time::now();
	sub_odom = n.subscribe("odom", 1, &Navigation::robotPosCallback, this);
	sub_scan = n.subscribe("scan", 1, &Navigation::scannerCallback, this);
	drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	effort_pub = n.advertise<brain2::MotorCommand>("motor_command", 1);


}

Navigation::~Navigation() {
	// TODO Auto-generated destructor stub
}

void Navigation::nextcmdCallback(const std_msgs::Bool cmd) {
	next_cmd = cmd.data;
}

void Navigation::robotPosCallback(
		const geometry_msgs::PoseWithCovarianceStamped &odom) {
	robotPose.x = odom.pose.pose.position.x;
	robotPose.y = odom.pose.pose.position.y;
	tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robotPose.theta = yaw;
	init = true;
	ROS_INFO("I'm at the position:[%f, %f, %f]", robotPose.x, robotPose.y,
			robotPose.theta * 180 / M_PI);
	if(ros::Time::now().sec-time_pose.sec>60&&!dangerGoalOn){
		time_pose = ros::Time::now();
		double diff;
		diff = pow(lastPose.y - robotPose.y, 2)
								+ pow(lastPose.x - robotPose.x, 2);
		if(sqrt(diff)<0.1){
			dangerGoal.x = lastPose.x;
			dangerGoal.y = lastPose.y;
			dangerGoalOn = true;
		}
		lastPose = robotPose;
	}

	if(ros::Time::now().sec-time_pose.sec>60){
		dangerGoalOn = false;
	}

}

void Navigation::scannerCallback(const sensor_msgs::LaserScan &scan) {
	if (!init) {
		ROS_INFO("NOT Waiting for Initialpose for 3sec");
		//sleep(3);
		if (init)
			ROS_INFO("Got initial pose start scanning (%f.%f.%f)", robotPose.x,
					robotPose.y, robotPose.theta);
		else
			ROS_WARN(
					"scannerCallback: No initial pose start scanning with Pose (0.0,0.0,0.0)");
		init = true;
	} //if not init
	ROS_INFO("--------------------------------------------");
	ROS_INFO("------------- Getting new Scan. ------------\n");
	angle_increment = scan.angle_increment;
	//ROS_INFO("angle_increment : %f",angle_increment);
	angle_min = scan.angle_min;
	ROS_INFO("angle_min : %f", angle_min);
	angle_max = scan.angle_max;
	//ROS_INFO("angle_max : %f",angle_max);
	actScan = scan.ranges;
	//ROS_INFO("scan_range: %lu",actScan.size());
	range_min = scan.range_min;
	//ROS_INFO("range_min: %f",range_min);
	range_max = scan.range_max;
	//ROS_INFO("range_max: %f",range_max);
	//count += 1;
	//if(count%2==1){
	//next_cmd =false;

//	std::vector<Navigation::safeSector> sectorList = getSectors(actScan);
//	localGoal = getLocalGoal(sectorList);
//	cout<<localGoal.x<<", "<<localGoal.y << endl;
//	double delta_alpha = (fmod(localGoal.y - robotPose.theta + 3 * M_PI,
//			2 * M_PI) - M_PI);
//	if (fabs(delta_alpha) < turn_threshold && !change_direct) {
//		drive(0.35);
//	} else {
//		if (delta_alpha > 0) {
//			rotate(0.25);
//		} else {
//			rotate(-0.25);
//		}
//	}
//
		ROS_INFO("rot_done: %s",rot_done ? "true":"false");
		double length2goal=sqrt(pow(Goal.x-robotPose.x ,2)
				+pow(Goal.y-robotPose.y,2));

		   if(rot_done){
				if(ros::Time::now().sec-time_start.sec>0.7&&driveOn){
					drive(0);
					driveOn=false;
				std::vector<Navigation::safeSector> sectorList = getSectors(actScan);
				localGoal = getLocalGoal(sectorList);
				//rotate(6);
				ROS_INFO("localGoal.x: %f",localGoal.x);
				ROS_INFO("localGoal.y: %f",localGoal.y*180/M_PI);
				//std::string temp;
				//std::cin>>temp;
				rot_done = false;
				}
			}
			else{
				//fmod(phi+3pi,2pi)-pi
				ROS_INFO("change_direct: %s",change_direct ? "true":"false");
//				double delta_alpha = (fmod(localGoal.y - robotPose.theta + 3 * M_PI,
//							2 * M_PI) - M_PI);
				double delta_alpha = (fmod(localGoal.y - robotPose.theta + 3 * M_PI,
											2 * M_PI) - M_PI);
//				double a=fabs(localGoal.y/fabs(localGoal.y)*M_PI-localGoal.y);
//				double b=fabs(robotPose.theta/fabs(robotPose.theta)*M_PI-robotPose.theta);
//				double rot_angle = fabs(a-robotPose.theta*localGoal.y*b/fabs(robotPose.theta)/fabs(localGoal.y));
//				ROS_INFO("rot_angle: %f",rot_angle );
				//ROS_INFO("robotPose.theta: %f",robotPose.theta);
				//ROS_INFO("delta_alpha: %f",localGoal.y - robotPose.theta);
				//std::string temp;
				//std::cin>>temp;
				if(fabs(delta_alpha)<turn_threshold||
				fabs(delta_alpha)>1.85*M_PI){
					rotate(0);
					ROS_INFO("stop ratate");
					//std::string temp;
					//std::cin>>temp;
					rot_done = true;
					if(!change_direct&&!driveOn){
						drive(0.3);
						driveOn = true;
						time_start = ros::Time::now();
					}else{
						change_direct=false;
						driveOn = true;
						time_start = ros::Time::now();
					}



				}
				else if(delta_alpha<0){
					if(ros::Time::now().sec-time_rotl.sec>1){
						//rotate(0);
						rotate(-4);
					time_rotl = ros::Time::now();
					}
				}

				else if(delta_alpha>0)
					if(ros::Time::now().sec-time_rotr.sec>1){
						//rotate(0);
						rotate(4);
						time_rotr = ros::Time::now();
					}
			}

	//   actScan_last =actScan;

}

Navigation::goal Navigation::getLocalGoal(
		std::vector<Navigation::safeSector> sector_list) {
	Navigation::goal lg; //local goal
	std::vector<double> theta_i;
	double d_min;
	double theta;
	ROS_INFO("getLocalGoal");
	for (int i = 0; i < sector_list.size(); i++) {
		ROS_INFO("sector_list[%d] idx:[%d, %d],[%f %f]", i,
				sector_list[i].SAS_idx, sector_list[i].SAE_idx,
				sector_list[i].SAS_dist, sector_list[i].SAE_dist);
		if (i == 0 && sector_list[i].SAS_idx == 0
				&& sector_list[i].SAE_idx < 20) {
			theta_i.push_back(0);
		}
		if (i == sector_list.size() - 1 && sector_list[i].SAS_idx > 490
				&& sector_list[i].SAE_idx == 511) {
			theta_i.push_back(M_PI);
		}

		if (sector_list[i].SAS_dist < sector_list[i].SAE_dist)
			d_min = sector_list[i].SAS_dist;
		else
			d_min = sector_list[i].SAE_dist;
		double theta = angle_increment
				* (sector_list[i].SAE_idx - sector_list[i].SAS_idx);
		double d = R * theta;
		ROS_INFO("d= %f", d);
		double diff = fabs(sector_list[i].SAE_idx - sector_list[i].SAS_idx);
		if (d >= D_safe && d <= 2 * D_safe) {

			theta_i.push_back(
					0.5
							* (angle_increment
									* (sector_list[i].SAS_idx
											+ sector_list[i].SAE_idx)));
		} else if (d > 2 * D_safe && d <= 4 * D_safe) {
			double angle_start = sector_list[i].SAS_idx + angle_offset
					+ 0.25 * diff;
			double angle_end = sector_list[i].SAS_idx - angle_offset
					+ 0.75 * diff;
			int angles = (int) ((angle_end - angle_start) / 20);
			for (int i = 0; i <= angles; i++) {
				theta_i.push_back(angle_increment * (angle_start + i * 20));
			}
		} else if (d > 4 * D_safe) {
			double angle_start = sector_list[i].SAS_idx + angle_offset
					+ diff / 6;
			double angle_end = sector_list[i].SAS_idx - angle_offset
					+ diff / 6 * 5;
			int angles = (int) ((angle_end - angle_start) / 20);
			for (int i = 0; i <= angles; i++) {
				theta_i.push_back(angle_increment * (angle_start + i * 20));
			}
		}
	}
	// theta_i in global coor. rechnen
	for (int i = 0; i < theta_i.size(); i++) {
		theta_i[i] = fmod(-theta_i[i] + M_PI / 2 + robotPose.theta, M_PI);
	}
	//calculate the cost function and choose the direction with the lowest cost
	double final_theta; //final chose direction6*
	double g_cost_last = 1000.0;
	double g_cost = 0.0;
	double aim_direct;
	aim_direct = atan2(-Goal.y + robotPose.y, Goal.x - robotPose.x);
	ROS_INFO("aim_direct: %f", aim_direct * 180 / M_PI);
	current_direct = robotPose.theta;

	/*if(theta_i.size()==0){
	 //double temp =(M_PI*3/2+robotPose.theta)%M_PI;
	 //final_theta = fmod((M_PI/3+robotPose.theta),M_PI);
	 final_theta = -robotPose.theta;
	 ROS_INFO("change direction,final_theta:%f",final_theta);
	 }*/
	if (theta_i.size() == 0) {
		final_theta = fmod(robotPose.theta+(rand()*2-1)*M_PI,M_PI);
//		final_theta = robotPose.theta
//						- robotPose.theta / fabs(robotPose.theta) * M_PI;
		ROS_INFO("change direction,final_theta:%f", final_theta);
		change_direct = true;
	} else {
		double diff=pow(dangerGoal.y - robotPose.y, 2)
						+ pow(dangerGoal.x - robotPose.x, 2) ;
		if (dangerGoalOn){
		ROS_INFO("danger_dist^2:%f",diff);
			double danger_direct = atan2(dangerGoal.y - 0,
					dangerGoal.x - 0);
			for (int i = 0; i < theta_i.size(); i++) {
				//ROS_INFO("theta_i[%d] in rad: %f",i,theta_i[i]);
				ROS_INFO("theta_i[%d] in grad: %f", i, theta_i[i] * 180 / M_PI);
				if (theta_i[i] - aim_direct < -M_PI) {
					g_cost = cost1 * fabs((theta_i[i] - aim_direct) + 2 * M_PI)
							- cost2 * abs(theta_i[i] - danger_direct);
				} else if (theta_i[i] - aim_direct > M_PI) {
					g_cost = cost1 * fabs((theta_i[i] - aim_direct) - 2 * M_PI)
							- cost2 * abs(theta_i[i] - danger_direct);
				} else {
					g_cost = cost1 * fabs(theta_i[i] - aim_direct)
							- cost2 * abs(theta_i[i] - danger_direct);
					;
				}
				ROS_INFO("g_cost: %f", g_cost);
				if (g_cost < g_cost_last) {
					final_theta = theta_i[i];
					g_cost_last = g_cost;
				}
			}
		} else {
			for (int i = 0; i < theta_i.size(); i++) {
				//ROS_INFO("theta_i[%d] in rad: %f",i,theta_i[i]);
				ROS_INFO("theta_i[%d] in grad: %f", i, theta_i[i] * 180 / M_PI);
				//g_cost = cost1*fabs(fabs(theta_i[i])-fabs(aim_direct));//+cost2*abs(theta_i[i]-current_direct)+cost3*abs(theta_i[i]-last_direct);
				if (theta_i[i] - aim_direct < -M_PI) {
					g_cost = cost1 * fabs((theta_i[i] - aim_direct) + 2 * M_PI); //+cost2*abs(theta_i[i]-current_direct)+cost3*abs(theta_i[i]-last_direct);
				} else if (theta_i[i] - aim_direct > M_PI) {
					g_cost = cost1 * fabs((theta_i[i] - aim_direct) - 2 * M_PI); //+cost2*abs(theta_i[i]-current_direct)+cost3*abs(theta_i[i]-last_direct);
				} else {
					g_cost = cost1 * fabs(theta_i[i] - aim_direct);
				}
				ROS_INFO("g_cost: %f", g_cost);
				if (g_cost < g_cost_last) {
					final_theta = theta_i[i];
					g_cost_last = g_cost;
				}
			}
		}
	}

	ROS_INFO("Chose direction: %f", final_theta * 180 / M_PI);
	//lg.x = robotPose.x + R*cos(final_theta+robotPose.theta);
	//lg.y = robotPose.y + R*sin(final_theta+robotPose.theta);
	lg.x = R;
	lg.y = final_theta;
	last_direct = robotPose.theta;
	if (fabs(final_theta - aim_direct) > M_PI / 2)
		thr_small = false;
	else
		thr_small = true;
	if (final_theta == fmod(M_PI / 2 + robotPose.theta, M_PI)
			|| final_theta == fmod(M_PI + M_PI / 2 + robotPose.theta, M_PI))
		change_direct = true;

	ROS_INFO("Local goal in global coor.: %f m, %f rad.", lg.x, lg.y);
	return lg;
}

std::vector<Navigation::safeSector> Navigation::getSectors(
		const std::vector<float> actScan) {

	std::vector<Navigation::safeSector> sector_list;
	Navigation::safeSector ss;
	ss.SAS = false;
	/*ROS_INFO("getSectors");
	 for (int i = 0; i < actScan.size(); i++){
	 ROS_INFO("actScan[%d]: %f", i, actScan[i]);
	 }*/

	ROS_INFO("scan data size: %lu", actScan.size());
	for (int i = 0; i < actScan.size(); i++) {//ROS_INFO("ss.SAS= %d i>=(actScan.size()-2) = %d",ss.SAS,i>=(actScan.size()-1));

		if (std::isnan(actScan[i]) || !std::isfinite(actScan[i])) {
			continue;
		}
		if (actScan[i] > M && ss.SAS == false)

		{
			ss.SAS = true;
			ss.SAS_idx = i;
			ss.SAS_dist = actScan[i];
			ROS_INFO("sector start");
		}
		if (actScan[i] > M && ss.SAS == true) {
			//ROS_INFO("sector start:  Ueberfluessig");
		}
		if (actScan[i] < M && ss.SAS == true) {
			//ROS_INFO("sector start:  true");
			if (i - ss.SAS_idx < 20 && ss.SAS_idx != 0) {
				ss.SAS = false;
			} else {
				ss.SAE_idx = i;
				ss.SAE_dist = actScan[i];
				sector_list.push_back(ss);
				ss.SAS = false;
			}
			ROS_INFO("sector closed.");
		}
		if (actScan[i] < M && (i - ss.SAE_idx < 5) && ss.SAS == false) {
			//ROS_INFO("sector start:  true");
			if (actScan[i] < ss.SAE_dist) {
				ss.SAE_dist = actScan[i];
				sector_list.pop_back();
				sector_list.push_back(ss);
			}
		}
		if (ss.SAS == true && i >= (actScan.size() - 1)) {
			//ROS_INFO("no sector found, sector end is set to last scan index!");
			ss.SAE_idx = i;
			ss.SAE_dist = R;
			sector_list.push_back(ss);
			ss.SAS = false;
			ROS_INFO("sector closed.");
		}
	}
	ROS_INFO("Number of sectors: %lu", sector_list.size());
	return sector_list;
}

void Navigation::drive2Goal(Navigation::goal local_goal) {
	if (local_goal.y == M_PI) {
		rotate_angle = local_goal.y;
		drive_dist = 0;

	} else {
		rotate_angle = local_goal.y;
		drive_dist = local_goal.x;
	}

	//double goal_direct = atan2(local_goal.y - robotPose.y,local_goal.x - robotPose.x);
	//double temp;
	//rotate_angle = robotPose.theta - goal_direct;
	//rotate(temp);
	/* while(1){
	 if(robotPose.theta - goal_direct < -turn_threshold ){
	 rotate(-0.1);
	 }
	 else if(robotPose.theta - goal_direct > turn_threshold ){
	 rotate(0.1);
	 }
	 else
	 break;// richtig ausgerichtet
	 } */
	//drive_dist = sqrt(pow(local_goal.y - robotPose.y,2)+pow(local_goal.y - robotPose.y,2));
	//drive(drv);

}

bool Navigation::goal_reached(Navigation::goal g) {
	//ROS_INFO("POSE:%f,%f,%f, GOAL%f,%f", robotPose.x, robotPose.y,
		//	robotPose.theta * M_PI / 180, g.x, g.y);

	return (fabs(robotPose.x - g.x) < drive_tol
			&& fabs(robotPose.y - g.y) < drive_tol); //different method because of angle
}

brain2::MotorCommand Navigation::pubStop() {
	brain2::MotorCommand cmd;
	cmd.name = "stop";
	cmd.r_effort = 0;
	cmd.l_effort = 0;
	return cmd;
}

void Navigation::rotate(double rot) {
	ROS_INFO("ROTATE %f", rot * 180.0 / M_PI);
	ROS_INFO("ROTATE %f", rot);
	geometry_msgs::Twist msg;

	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = rot;

	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;

	drive_pub.publish(msg);

	//sleep(2);
}

void Navigation::drive(double drive) {
	ROS_INFO("DRIVE %f", drive);
	geometry_msgs::Twist msg;

	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;

	msg.linear.x = drive;
	msg.linear.y = 0;
	msg.linear.z = 0;
	drive_pub.publish(msg);

	//sleep(2);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Navigation");
	ros::NodeHandle n;

	Navigation m = Navigation(n);
	while (ros::ok()) {
		ros::Rate loop_rate(20);
		//ROS_INFO("next_cmd: %s \n",m.next_cmd ? "true":"false");
		ros::spinOnce();
		loop_rate.sleep();

		if (m.goal_reached(m.Goal)) {
			m.drive(0);
			ros::shutdown();
			break;
		}
		//sleep(10);
	}
	//ros::shutdown();
	//os::spin();
	return 0;
}
