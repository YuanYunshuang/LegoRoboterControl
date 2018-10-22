/*
 * Scananalyzer.cpp
 *
 *  Created on: Feb 22, 2018
 *      Author: ophelia
 */

#include "Scananalyzer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <iostream>
#include <list>
#include <limits>
#include <math.h>
#include <laser_geometry/laser_geometry.h>
//#include "Navigation.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

//
Scan_analyzer::Scan_analyzer() {
	// TODO Auto-generated constructor stub
	init = false;
	scan_min = 10;
	D_safe = 0.3; //Safe width that the robot can pass through
	isSafe_r = true;
	isSafe_f = true;
	isSafe_l = true;
	angle_increment = M_PI/512;
	angle_min = 0;
	angle_max = M_PI;
	range_min = 0;
	robotPose.x = 0.0;
	robotPose.y = 0.0;
	robotPose.theta = 0.0;
	//**********************************Map Initializing****************************
	old_x= 0;  old_y = 0;
	new_x= 0;  new_y = 0;
	// NICHT NEGATIV mit FABS
	map = cv::Mat(60,60,CV_64F,100.0);
	//******************************************************************************

}

void Scan_analyzer::refresh(const sensor_msgs::LaserScan &scan,
		geometry_msgs::Pose2D robotPoseNew){
	if(!init){
//		actScan = scan.ranges;
		actScan = smoothScan(scan.ranges);
		std::cout<<"scan data initialized."<<std::endl;
		robotPose.x = robotPoseNew.x;
		robotPose.y = robotPoseNew.y;
		robotPose.theta = robotPoseNew.theta;
	/*	//**********************************Map Initializing****************************
		old_x = fabs(ceil(robotPose.x*20))+5;
		old_y = fabs(ceil(robotPose.y*20))+5;
		for(int i=-2;i<3;i++){
			for(int j=-2;j<3;j++){
				if(map.at<double>(old_x+i, old_y+j)>99){
					map.at<double>(old_x+i, old_y+j) = 0;
				}
				map.at<double>(old_x+i, old_y+j) = map.at<double>(old_x+i, old_y+j)
				                                   + 3-std::max(abs(i),abs(j));
			}
		}
		//******************************************************************************/
		init = true;
	}
	else{
		actScan_last = actScan;
//		actScan = scan.ranges;
		actScan = smoothScan(scan.ranges);
		//std::cout<<"scan data refreshed."<<std::endl;
	}
		std::cout<<"--------------------------------------------"<<std::endl;
		std::cout<<"------------- Getting new Scan. ------------"<<std::endl;
		angle_increment = scan.angle_increment;
		angle_min = scan.angle_min;
		angle_max = scan.angle_max;
		range_min = scan.range_min;
		range_max = scan.range_max;
		robotPose.x = robotPoseNew.x;
		robotPose.y = robotPoseNew.y;
		robotPose.theta = robotPoseNew.theta;
		//std::cout<<"scan data variables and robotPose refreshed."<<std::endl;
//		Scan_analyzer::sector_IsSafe(1,0.15);
//		std::cout<<"Function:sector_IsSafe done."<<std::endl;
//		Scan_analyzer::updateMap();
//		std::cout<<"Function:updateMap done."<<std::endl;
//		Scan_analyzer::updateMapPose();
//		std::cout<<"Function:updateMapPose done."<<std::endl;
}

Scan_analyzer::~Scan_analyzer() {
	// TODO Auto-generated destructor stub
}

std::vector<Scan_analyzer::safeSector> Scan_analyzer::getSectors(double radius){
	Scan_analyzer::safeSector ss;
	ss.SAS = false;
	for(int i=0; i<actScan.size();i++){
		if (std::isnan(actScan[i]) || !std::isfinite(actScan[i])) {
			continue;
		}
		if (actScan[i] >radius  && ss.SAS == false)

		{
			ss.SAS = true;
			ss.SAS_idx = i;
			ss.SAS_dist = actScan[i];
			ROS_INFO("sector start");
		}
		if (actScan[i] > radius && ss.SAS == true)
		if (actScan[i] < radius && ss.SAS == true) {
			if (i - ss.SAS_idx < 20 && ss.SAS_idx != 0) {
				ss.SAS = false;
			} else {
				ss.SAE_idx = i;
				ss.SAE_dist = actScan[i];
				sector_list.push_back(ss);
				ss.SAS = false;
			}
		}
		if (actScan[i] < radius && (i - ss.SAE_idx < 5) && ss.SAS == false) {
			if (actScan[i] < ss.SAE_dist) {
				ss.SAE_dist = actScan[i];
				sector_list.pop_back();
				sector_list.push_back(ss);
			}
		}
		if (ss.SAS == true && i >= (actScan.size() - 1)) {
			ss.SAE_idx = i;
			ss.SAE_dist = radius;
			sector_list.push_back(ss);
			ss.SAS = false;
			ROS_INFO("sector closed.");
		}
	}
	return sector_list;
}

/* the scan angle is divided in 3 sectors, left, front and right,
 * if the sector is safe to drive, the correspondent variable will
 * be set to true.
 */
bool Scan_analyzer::sector_IsSafe(int sector, double r){
	bool isSafe_r = true;
	bool isSafe_f = true;
	bool isSafe_l = true;

	for (int i = 0; i < actScan.size(); i++) {
		if(actScan[i]<r && i<= (int) floor(actScan.size()/3))
		{
			isSafe_r = false;
		}else if(actScan[i]<r && i<= (int) floor(actScan.size()*2/3))
		{
			isSafe_f = false;
		}else if(actScan[i]<r && i<= (int) floor(actScan.size()))
		{
			isSafe_l = false;
		}
	}
	std::cout<<"front :"<< isSafe_f<<std::endl;
	switch(sector){
	case 1: return isSafe_r;
	case 2: return isSafe_f;
	case 3: return isSafe_l;
	default: {
		std::cout<<"Please pass the right sector argument!\n 1=right,2=front,3=left."<<std::endl;
		return false;
	}

	}
}

/* Calculate the minimal scan distance from one scan
 *
 */
double Scan_analyzer::getMinDist(){
	scan_min =10;
	for(int i=0; i<actScan.size();i++){
		if(actScan[i]<scan_min){
			scan_min = actScan[i];
		}
	}
}
/* Get possible angles that robot can run at next step, the angles are given in
 * global coordinates frame.
 */
std::vector<double> Scan_analyzer::getSafeAngles(double radius){
		Scan_analyzer::getSectors(radius);
		std::vector<double> theta_i;
		double d_min;
		double theta;
		ROS_INFO("getLocalGoal");
		sector_list.clear();
		for (int i = 0; i < sector_list.size(); i++) {
			ROS_INFO("sector_list[%d] idx:[%d, %d],[%f %f]", i,
					sector_list[i].SAS_idx, sector_list[i].SAE_idx,
					sector_list[i].SAS_dist, sector_list[i].SAE_dist);

			double diff = fabs(sector_list[i].SAE_idx - sector_list[i].SAS_idx);
			if (sector_list[i].SAS_dist < sector_list[i].SAE_dist)
				d_min = sector_list[i].SAS_dist;
			else
				d_min = sector_list[i].SAE_dist;
			double d_theta = angle_increment* diff;
			double d = radius * d_theta;
			ROS_INFO("d= %f", d);

			if (d >= D_safe && d <= 2 * D_safe) {
				theta_i.push_back(0.5* (angle_increment* (sector_list[i].SAS_idx+ sector_list[i].SAE_idx)));
			} else if (d > 2 * D_safe && d <= 4 * D_safe) {
				double angle_start = sector_list[i].SAS_idx + angle_offset+ 0.25 * diff;
				double angle_end = sector_list[i].SAS_idx - angle_offset+ 0.75 * diff;
				int angles = (int) ((angle_end - angle_start) / 20);
				for (int i = 0; i <= angles; i++) {
					theta_i.push_back(angle_increment * (angle_start + i * 20));
				}
			} else if (d > 4 * D_safe) {
				double angle_start = sector_list[i].SAS_idx + angle_offset+ diff / 6;
				double angle_end = sector_list[i].SAS_idx - angle_offset+ diff / 6 * 5;
				int angles = (int) ((angle_end - angle_start) / 20);
				for (int i = 0; i <= angles; i++) {
					theta_i.push_back(angle_increment * (angle_start + i * 20));
				}
			}
		}
		// theta_i in global coor. rechnen
		for (int i = 0; i < theta_i.size(); i++) {
			theta_i[i] = fmod(theta_i[i] + robotPose.theta, 2*M_PI);
		}
		return theta_i;
}

std::vector<float> Scan_analyzer::smoothScan(std::vector<float> actScan){
	double start = 0;
	double dist;
	bool countOn = false;
	int count = 0;
	for(int i = 1;i<actScan.size(); i++){
		if(actScan[i]-actScan[i-1]>0.5 && !countOn){
			start = i-1;
			//std::cout<<"smoothScan start:"<<start<<std::endl;
			dist = actScan[i-1];
			std::cout<<"smoothScan dist:"<<dist<<std::endl;
			countOn = true;
			for(int j=1;j<10;j++){
				if(start+j>0&&start+j<512)
				actScan[start+j]=dist;
			}
		}
		if(countOn){
			count++;
			//std::cout<<count<<std::endl;
		}
		if(countOn && actScan[i-1]-actScan[i]>0.5){
			std::cout<<"smoothScan ended:"<<start<<","<<i<<std::endl;
			countOn = false;
			//std::cout<<"count*angle_increment*dist:"<<count*angle_increment*dist<<std::endl;
			if(count*angle_increment*dist<0.3){
				for(int ct=start; ct<start+count;ct++){
					actScan[ct] = dist;
				}
			}
//			else{
//				for(int k=1;k<2;k++){
//					if(i-k>0&&i-k<512)
//					actScan[i-k]=dist;
//				}
//			}
			count = 0;
		}

	}
//	for(int i = 1;i<actScan.size(); i++){
//		if(actScan[i]>1){
//			actScan[i] = 1;
//		}
//	}
	return actScan;
}

/*Update the environment in the map*/
void Scan_analyzer::updateMap(){
	double xs_b, ys_b;//scanning points in body-frame
	double xs_g, ys_g;//scanning points in global-frame
	//set empty points on the laser beam to zero
	for(int i = 0;i<actScan.size(); i++){
		//std::cout<<actScan[i]<<",";
		if (std::isnan(actScan[i]) || !std::isfinite(actScan[i])) {
			continue;
		}
		else{
			for(int j=0;j<=floor(actScan[i]/0.05)+1;j++){
				//std::cout<<"actScan["<<i<<"]="<<actScan[i]<<std::endl;
				xs_b = j*0.05*cos(i*angle_increment);
				ys_b = j*0.05*sin(i*angle_increment)+0.075;//offset from scanner- to body-frame
				xs_g = cos(-robotPose.theta)*xs_b-sin(-robotPose.theta)*ys_b+robotPose.x;
				ys_g = sin(-robotPose.theta)*xs_b+cos(-robotPose.theta)*ys_b+robotPose.y;
				double map_x = fabs(ceil(xs_g*20))+5;
				double map_y = fabs(ceil(ys_g*20))+5;
				if(map_x>5&&map_x<48&& map_y>5&&map_y<48){
					if(j>floor(actScan[i]/0.05)&&map.at<double>(map_x, map_y)==0.0){
						map.at<double>(map_x, map_y)=100.0;
					}

					else if(map.at<double>(map_x, map_y)==100.0){
						map.at<double>(map_x, map_y)=0.0;
					}
				}
		    }
	    }
    }
	std::cout<<std::endl;
	//print the map
	for (int j = 0; j<60; j++)
	{
		for(int i = 0;i<60; i++ ){
			int x = j;
			int y = i;
			if(int(map.at<double>(x, y))>=200.0){
				//std::cout<<std::setw(2)<<"+";
				std::cout<<"o ";
			}
			else if(int(map.at<double>(x, y))>=100.0){
				//std::cout<<std::setw(2)<<"+";
				std::cout<<"+ ";
			}
			else if(int(map.at<double>(x, y))>0.0){
				std::cout<<". ";
			}
			else{
				std::cout<<"  ";
			}

		}
		std::cout<<std::endl;
	}
	double min,max;
	cv::minMaxIdx(map,&min,&max);
	cv::Mat tmp=cv::Mat::zeros(map.cols,map.rows,CV_8UC1);

			cv::convertScaleAbs(map,tmp,180/max);
			cv::resize(tmp,tmp,cv::Size(1000,1000));
			cv::circle(tmp,cv::Point2d(robotPose.x/20+5,robotPose.y/20+5),1,cv::Scalar(255),1);
	cv::imshow("map",tmp);
	cv::waitKey(100);
	//std::string temp;
	//std::cin>>temp;
}

/*Update the robot position in the map by adding 1 , map origin is at left bottom*/
void Scan_analyzer::updateMapPose(){

		new_x = fabs(ceil(robotPose.x*20))+5;
		new_y = fabs(ceil(robotPose.y*20))+5;
		std::cout<<"robotPose in map: "<<new_x<<","<<new_y<<std::endl;

		if(new_x != old_x || new_y != old_y)
		{
			for(int i=-4;i<5;i++){
				for(int j=-4;j<5;j++){
					if(new_x+i>5&&new_x+i<48&&new_y+j>5&&new_y+j<48){
//						if(map.at<double>(new_x+i, new_y+j)>99&&std::max(abs(i),abs(j))<3){
//							map.at<double>(new_x+i, new_y+j) = 0;
//						}
						map.at<double>(new_x+i, new_y+j) = map.at<double>(new_x+i, new_y+j)
														   + 3-std::max(abs(i),abs(j))*0.5;
						old_x = new_x;
						old_y = new_y;
					}
				}
			}
		}
}
