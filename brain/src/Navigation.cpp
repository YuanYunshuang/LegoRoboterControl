/*
 * Navigation.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: ophelia
 */
#include "Navigation.h"

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
#include <opencv2/core.hpp>
#include <stdlib.h>
#include<cstdlib>
#include <brain/SensorData.h>
#include "Scananalyzer.h"
#include<time.h>

//////////////////////////////

bool rot_done = true;
bool drv_done = true;
bool change_direct = false;
bool driveOn = true;
bool nextGoal = true;
bool routeErr = false;
bool bCalNewLane = true;
bool ini_lp = false; //if lastPose initialized?
int mapUpdate = 0;
int updatect = 0;
double time_start;
double time_rotl;
double time_rotr;
double time_pose;
double time_mapUpdate;
double aim_angle;
bool total_init_done = false;
//////////////////////////////

Navigation::Navigation(ros::NodeHandle n, Scan_analyzer sa) {
	// TODO Auto-generated constructor stub
	n_ = n;
	time_start = ros::Time::now().toSec();
	time_rotl = ros::Time::now().toSec();
	time_rotr = ros::Time::now().toSec();
	time_pose = ros::Time::now().toSec();
	time_mapUpdate=ros::Time::now().toSec();
	routeOn =false;
	scan_analyzer = sa;
	turn_threshold = 0.15;
	turn_step = 0.3;
	drive_tol = 0.1;
	angle_offset = 20;
	Dij_map = cv::Mat(25,25,CV_64F,0.0);
	std::cin >> Goal.x;
	std::cin >> Goal.y;
	cost1 = 20;
	cost2 = 1;
	localGoal.x = 0.0;
	localGoal.y = 0.0;
	robotPose.x = 0.0;
	robotPose.y = 0.0;
	robotPose.theta = 0.0;
	lastPose.x = 0.0;
	lastPose.y = 0.0;
	lastPose.theta = 0.0;
	init = false;

	//tracking_pose_delay = ros::Time::now().toSec();
	sub_odom = n.subscribe("odom", 1, &Navigation::robotPosCallback, this);
	sub_scan = n.subscribe("scan", 1, &Navigation::scannerCallback, this);
	sub_ultra = n.subscribe("sensor_data", 1, &Navigation::ultraCallback, this);
	drive_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	effort_pub = n.advertise<brain::MotorCommand>("motor_command", 1);


}

Navigation::~Navigation() {
	// TODO Auto-generated destructor stub
}



void Navigation::robotPosCallback(
		const geometry_msgs::PoseWithCovarianceStamped &odom) {
	time_pose = ros::Time::now().toSec();
	robotPose.x = odom.pose.pose.position.x;
	robotPose.y = odom.pose.pose.position.y;
	/*tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);*/
	robotPose.theta = odom.pose.pose.position.z;
	init = true;
	ROS_INFO("I'm at the position:[%f, %f, %f]", robotPose.x, robotPose.y,
			robotPose.theta * 180 / M_PI);
	if(!ini_lp){
		lastPose = robotPose;
		ini_lp = true;
	}
	scan_analyzer.updateMapPose();
	/*if(pow(robotPose.x-lastPose.x,2)+pow(robotPose.y-lastPose.y,2)>0.04){
		lastPose = robotPose;
		Navigation::goal g2 = getLocalGoal();
		Navigation::goal g1;
		g1.x =robotPose.x;
		g1.y =robotPose.y;
		Navigation::lane ln = getLane(g1,g2);


	}*/
}

void Navigation::ultraCallback(const brain::SensorData ultra){
	double xs_b, ys_b;//scanning points in body-frame
	double xs_g, ys_g;//scanning points in global-frame
	if(ultra.distance<20){
		ultraPass = false;
//		xs_b = ultra.distance*cos(M_PI/2)/100;
//		ys_b = (ultra.distance-2)*sin(M_PI/2)/100+0.1;//offset from scanner- to body-frame
//		xs_g = cos(-robotPose.theta)*xs_b-sin(-robotPose.theta)*ys_b+robotPose.x;
//		ys_g = sin(-robotPose.theta)*xs_b+cos(-robotPose.theta)*ys_b+robotPose.y;
//		double map_x = fabs(ceil(xs_g*20))+5;
//		double map_y = fabs(ceil(ys_g*20))+5;
//		if(map_x>5&&map_x<48&& map_y>5&&map_y<48){
//				scan_analyzer.map.at<double>(map_x, map_y)=105.0;
//		}
//		route.clear();
//		//rotate(0.2);
//		nextGoal = true;
//		if(fabs(robotPose.theta)-floor(fabs(robotPose.theta)*2/M_PI)<M_PI/4){
//			for(int x=-1;x<2;x++){
//				map_x = map_x + x;
//				if(map_x>5&&map_x<48&& map_y>5&&map_y<48){
//						scan_analyzer.map.at<double>(map_x, map_y)=100.0;
//				}
//			}
//		}
//		else{
//			for(int y=-1;y<2;y++){
//				map_y = map_y + y;
//				if(map_x>5&&map_x<48&& map_y>5&&map_y<48){
//						scan_analyzer.map.at<double>(map_x, map_y)=100.0;
//				}
//			}
//		}

	}

	else{
		ultraPass = true;
	}
}

void Navigation::scannerCallback(const sensor_msgs::LaserScan &scan) {
	scan_analyzer.refresh(scan,robotPose);
	if(ros::Time::now().toSec()-time_mapUpdate>2 && mapUpdate==0 && total_init_done){
		mapUpdate = 1;
		drive(0);
	}

	//std::cout<<init<<","<<mapUpdate<<std::endl;
	if(init && mapUpdate==0){
		if(nextGoal){
			if(!total_init_done){
				scan_analyzer.updateMap();
				goal lg ;
				lg.x = 0;
				lg.y = -0.2;
				lg.x = cos(-robotPose.theta)*lg.x-sin(-robotPose.theta)*lg.y+robotPose.x;
				lg.y = sin(-robotPose.theta)*lg.x+cos(-robotPose.theta)*lg.y+robotPose.y;
				std::cout<<"robotPose:"<<robotPose.x<<","<<robotPose.y<<std::endl;
				std::cout<<"lg:"<<lg.x<<","<<lg.y<<std::endl;
				route.push_back(lg);
				aim_angle = -robotPose.theta;
				//rot_done = false;
				nextGoal = false;
			}else{
				updatect++;
				std::cout<<"[175]"<<ros::Time::now().toSec()-time_pose<<","<<updatect<<std::endl;
				if(ros::Time::now().toSec()-time_pose<0.5 && updatect>15){
					scan_analyzer.updateMap();
					time_mapUpdate = ros::Time::now().toSec();
					updatect = 0;

					std::cout<<"Getting next goal and it's route..."<<std::endl;
					while(route.size()<1){
						route = getRoute();
						//std::cout<<"Cannot find route!";
					}

					route.erase(route.begin());
					//print route
					std::cout<<"route:";
					for(int i=0; i<route.size();i++){
						std::cout<<"["<<route[i].x<<","<<route[i].y<<"],";
					}
					std::cout<<std::endl;
					//std::string temp;
					//std::cin>>temp;
					//rot_done = false;
					//if(routeOn){
						nextGoal = false;
					//}
				}
			}


		}
		else{
			if(rot_done){
				std::cout<<"Driving to the local goal:"<<"["<<route[0].x<<","<<route[0].y<<"]"<<std::endl;
				drive2Goal(route[0]);
				if(!driveOn){
					aim_angle = atan2(route[0].x-robotPose.x,route[0].y-robotPose.y);
//					std::vector<double> safe_angles = scan_analyzer.getSafeAngles(0.3);
//					double chose_angle = aim_angle;
//					if(safe_angles.size()>0){
//						//Calculating costs
//						double cost = 100;
//						double old_cost = 100;
//						for(int i=0; i<safe_angles.size();i++){
//							double cost = fabs(fmod(aim_angle - safe_angles[i] + 3 * M_PI,
//																		2 * M_PI) - M_PI);
//							if(cost<old_cost){
//								chose_angle = safe_angles[i];
//								old_cost = cost;
//							}
//						}
//					}else{
//						chose_angle = robotPose.theta + M_PI*(std::rand()/(RAND_MAX + 1.0));
//						change_direct = true;
//					}


					ROS_INFO("aim angle: %f",aim_angle*180/M_PI);
//					ROS_INFO("chose angle: %f",chose_angle*180/M_PI);
//					std::string temp;
//					std::cin>>temp;
					rot_done = false;
				}
			}
			else{
				rotate2AimAngle(aim_angle);
			}
			if(goal_reached(localGoal) || route.size()<1){
				nextGoal = true;
				routeOn = false;
			}
		}
		//init = false;
	}
	else if(mapUpdate){
		std::cout<<"Waiting for robot pose or map update..."<<std::endl;
		mapUpdate++;
		//control the update based on a about 3 second static pose(30hz=1s)
		std::cout<<ros::Time::now().toSec()-time_pose<<","<<mapUpdate<<std::endl;
		if(ros::Time::now().toSec()-time_pose<0.5 && mapUpdate>15){
			scan_analyzer.updateMap();
			time_mapUpdate = ros::Time::now().toSec();
//			std::string temp;
//			std::cin>>temp;
			mapUpdate = 0;
		}
	}
}

Navigation::goal Navigation::getLocalGoal(){
	Navigation::goal localGoal;
	int count = 0;
	while(1){ //generate a std::random local goal where the robot has never been
		count++;
		int x = int(45*(std::rand()/(RAND_MAX + 1.0))+5);
		int y = int(45*(std::rand()/(RAND_MAX + 1.0))+5);
		int count1 = 0;
		for(int i=-2;i<3;i++){
			for(int j=-2;j<3;j++){
				if(scan_analyzer.map.at<double>(x+i, y+j)==0){
					count1++;
				}
			}
		}
		int count2 = 0;
		for(int i=-3;i<4;i++){
			for(int j=-3;j<4;j++){
				if(scan_analyzer.map.at<double>(x+i, y+j)<100){
					count2++;
				}
			}
		}
		int count3 = 0;
		for(int i=-2;i<3;i++){
			for(int j=-2;j<3;j++){
				if(scan_analyzer.map.at<double>(x+i, y+j)<100){
					count3++;
				}
			}
		}
		double diff = pow((x-5)/20-robotPose.x,2)+pow((y-5)/20-robotPose.y,2);

		if((count1==49 && count2==49 && diff>=0.04 && count<2000)){
			localGoal.x = double((x-5)/20.0);
			localGoal.y = double((y-5)/20.0);
			std::cout<<"localGoal in global coordinates:"<<localGoal.x<<","<<localGoal.y<<std::endl;
			break;
		}
		if((count3==25&& count1>=20 && count>2000)){
			localGoal.x = double((x-5)/20.0);
			localGoal.y = double((y-5)/20.0);
			std::cout<<"localGoal in global coordinates:"<<localGoal.x<<","<<localGoal.y<<std::endl;
			break;
		}

	}
	return localGoal;
}

/* in the returned route the start point
 * is stored at route[0] and the goal point in route.back().
 */
std::vector<Navigation::goal> Navigation::getRoute(){
	std::vector<Navigation::goal> route;
	routeOn =false;//no route to a local goal
	Navigation::goal g1;
	g1.x = robotPose.x;
	g1.y = robotPose.y;
	int count = 0;
	int x = fabs(ceil(Goal.x*20))+5;
	int y = fabs(ceil(Goal.y*20))+5;
	for(int i=-1;i<2;i++){
		for(int j=-1;j<2;j++){
			if(scan_analyzer.map.at<double>(x+i, y+j)<100.0){
				count++;
			}
		}
	}
	std::cout<<"count goal -------------"<<count<<std::endl;
//	std::string temp;
//	std::cin>>temp;

	if(count==9){
		Navigation::lane l = getLane(g1,Goal);
			if(l.noObj){
					route = l.mid_line;
					std::cout<<"Got direct lane to the localGoal."<<std::endl;
					routeOn = true;
					route.push_back(Goal);
					return route;
			}
			else{
				std::cout<<"Got direct lane failed, calculating Dijkstra path..."<<std::endl;
				refreshDijVertexList();
				std::cout<<"Got vertex list."<<std::endl;
				refreshLinksTable();
				std::cout<<"Got link table."<<std::endl;
				vertex vex1 = matchVertex(g1);
				vertex vex2 = matchVertex(Goal);
				route = getShortestPath(vex1, vex2);
				std::cout<<"Got Dij. route."<<std::endl;
				//routeOn = true;
			}
	}

	if(!routeOn){
		Navigation::goal g2 = getLocalGoal();
		//try if there is a direct lane to the local goal
		Navigation::lane l = getLane(g1,g2);
		if(l.noObj){
				route = l.mid_line;
				std::cout<<"Got direct lane to the localGoal."<<std::endl;
				route.push_back(g2);
		}
		else{
			std::cout<<"Got direct lane failed, calculating Dijkstra path..."<<std::endl;
			refreshDijVertexList();
			std::cout<<"Got vertex list."<<std::endl;
			refreshLinksTable();
			std::cout<<"Got link table."<<std::endl;
			vertex vex1 = matchVertex(g1);
			vertex vex2 = matchVertex(g2);
			route = getShortestPath(vex1, vex2);
			std::cout<<"Got Dij. route."<<std::endl;
		}
		routeOn = true;
	}

	return route;
}

/* search for the shortest path in the routing table, in the returned path the start point
 * is stored at path[0] and the goal point in path.back().
 */
std::vector<Navigation::goal> Navigation::getShortestPath(vertex v1, vertex v2){
	RT rt = getRouteTablefor(v1);
	std::cout<<"getRouteTablefor(v1) finished."<<std::endl;
	std::vector<Navigation::goal> path;
	int predecessor_id = v2.id;
	path.push_back(v2.coor);
	while(1){
		if(predecessor_id<rt.size()){
			predecessor_id = rt[predecessor_id].pred_id;
		}else{
			route.clear();
			break;
		}
		if(predecessor_id<vertex_list.size()){
			path.insert(path.begin(),vertex_list[predecessor_id].coor);
			if(predecessor_id==v1.id){
				break;
			}
		}else{
			route.clear();
			break;
		}
	}
	Navigation::goal temp;
	int it = 1;
	for(int i=0; i<path.size()-1; i++){
		double diff = pow(path[i+1].x-path[i].x,2)+pow(path[i+1].y-path[i].y,2);
		double n = floor(sqrt(diff)/0.2);

		double dx = (path[i+1].x-path[i].x)/n;
		double dy = (path[i+1].y-path[i].y)/n;
		if(diff>0.04){
			for(int j=1; j<n;j++){
				temp.x = dx*j+path[i].x;
				temp.y = dy*j+path[i].y;
				path.insert(path.begin()+it,temp);
				it++;
			}
		}
		it++;
	}
	if(path.size()>0){
		path.push_back(v2.coor);
		routeOn = true;

	}
	std::cout<<"now returning path, path size = "<<path.size()<<std::endl;
	return path;
}

Navigation::vertex Navigation::matchVertex(goal g){
	//double Dij_map_x = floor(g.x*4)*0.25+0.125;
	//double Dij_map_y = floor(g.y*4)*0.25+0.125;
	bool vexFound = false;
	std::cout<<"Dij_map_x,y: "<<g.x<<","<<g.y<<std::endl;
	vertex vex;
	for(int w=1;w<4;w++){
		if(!vexFound){
			for(int i=0;i<vertex_list.size();i++){
				if(fabs(vertex_list[i].coor.x-g.x)<=0.125*w &&
						fabs(vertex_list[i].coor.y-g.y)<=0.125*w){
					vex = vertex_list[i];
					vexFound = true;
					break;
				}
			}
		}
		if(vexFound){
			break;
		}
	}


	std::cout<<"vertex "<<vex.id<<" matched."<<std::endl;
	return vex;
}

Navigation::RT Navigation::getRouteTablefor(vertex v){
	std::cout<<"Getting route table for vertex "<<v.id<<std::endl;
	RT rt; // declare a new route table for vertex v
	RTcell cell;
	// initializing the route table
	for(int i=0;i<vertex_list.size();i++){
		cell.id = i;
		cell.cost = table_links.at<double>(v.id,i);
		cell.pred_id = v.id;
		cell.marked = false;
		if(v.id==i){
			cell.marked = true;
		}
		rt.push_back(cell);
	}
	//refresh the route table and get the final optimal table
	while(1){
		if(allVertexMarked(rt)){
			break;
		}
		RTcell min_cell = findMinCell(rt);
		//rt[min_cell.id].marked = true;
		for(int i=0;i<vertex_list.size();i++){
			if(!rt[i].marked){
				double new_cost = min_cell.cost + table_links.at<double>(min_cell.id,i);
				if(rt[i].cost>new_cost){
					rt[i].cost = new_cost;
					rt[i].pred_id = min_cell.id;
				}
			}
		}
	}
	return rt;
}


Navigation::RTcell Navigation::findMinCell(RT& rt){
	RTcell min_cell;
	min_cell.cost = 888;

	for(int i =0;i<rt.size();i++){
		if(!rt[i].marked && min_cell.cost>rt[i].cost){
			min_cell = rt[i];
		}
	}
	rt[min_cell.id].marked = true;
	min_cell = rt[min_cell.id];
	//std::cout<<"min_cell found."<<std::endl;
	return min_cell;
}

bool Navigation::allVertexMarked(RT rt){
	int count = 0;
	for(int i =0;i<rt.size();i++){
		if(!rt[i].marked){
			count++;
		}
	}
	if(count>0){
		//std::cout<<count<<" vertices not marked."<<std::endl;
		return false;
	}
	else{
	return true;
	}
}


/* Get the 2d point cloud for the lane between 2 points, the lane has the width of 0.3m,
 * which is a proximate width of the robot body.
 */
Navigation::lane Navigation::getLane(goal a, goal b){
	double length = sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
	Navigation::lane l;
	l.pstart = a;
	l.pend = b;
	l.length = length;
	l.noObj = true;
	goal g;
	double theta = atan2(b.y-a.y,b.x-a.x);
	for(int i=0;i<int(length/0.05)+1;i++){
		for(int j=-2;j<3;j++){
			g.x = i*0.05*cos(theta)-(j*0.05)*sin(theta) + a.x;
			g.y = i*0.05*sin(theta)+(j*0.05)*cos(theta) + a.y;
			double x = fabs(ceil(g.x*20))+5;
			double y = fabs(ceil(g.y*20))+5;
			l.points.push_back(g);
			if(j==3 && i%3==0){
				l.mid_line.push_back(g);
			}
			if(scan_analyzer.map.at<double>(x,y)>99){
				l.noObj = false;
				break;
			}
			if(!l.noObj){
				break;
			}
		}
	}
	return l;
}

/* Calculate navigation plan based on the new updated Dij_nodes.
 * Update the coordinates of Dijkstra nodes.
 */
void Navigation::refreshLinksTable(){

	table_links = cv::Mat(vertex_list.size(),vertex_list.size(),CV_64F,100.0);
	for(int i=0;i<vertex_list.size();i++){
		for(int j=0;j<vertex_list.size();j++){
			if(i!=j){
				Navigation::lane l = getLane(vertex_list[i].coor,vertex_list[j].coor);
				if(l.noObj){
					double dist = sqrt(pow(vertex_list[j].coor.x-vertex_list[i].coor.x,2)
							+pow(vertex_list[j].coor.y-vertex_list[i].coor.y,2));
					table_links.at<double>(i,j) = dist;
					//std::cout<<"link from "<<i<<" to "<<j<<" established."<<std::endl;
				}
			}
			else{
				table_links.at<double>(i,j) = 0;
			}
		}
	}
//	//print the link table
//	for (int i = 0;i<vertex_list.size(); i++)
//	{
//		for(int j = 0;j<vertex_list.size(); j++ ){
//			std::cout<<std::setw(8)<<table_links.at<double>(i, j)<<",";
//		}
//		std::cout<<std::endl;
//	}
}

/* Clear node in the current Dijkstra map.
 * Update the coordinates of Dijkstra nodes.
 * Dij_map.at<double>(i, j)=1 ---> activated Dij_node
 * Dij_map.at<double>(i, j)=0 ---> not activated Dij_node
 */
void Navigation::refreshDijVertexList(){
	bool set1;
	for(int i=0;i<25;i++){
		for(int j=0;j<25;j++){
			if(Dij_map.at<double>(i, j)<1){
				double x = fabs(ceil((i*0.1+0.1)*20))+5;
				double y = fabs(ceil((j*0.1+0.1)*20))+5;
				set1 = true;
				for(int m=-2;m<3;m++){
					for(int n=-2;n<3;n++){
						if(scan_analyzer.map.at<double>(x+m,y+n)>99){
							set1 = false;
							break;
						}
						if(!set1){
							break;
						}
					}
				}
				if(set1){
					Dij_map.at<double>(i, j) = 1;
				}
			}
		}
	}
	vertex_list.clear();
	Navigation::vertex vex;
	vex.coor.x = robotPose.x;
	vex.coor.y = robotPose.y;
	vex.id = vertex_list.size();
	vertex_list.push_back(vex);

	for(int i=0;i<25;i++){
		for(int j=0;j<25;j++){
			if(Dij_map.at<double>(i, j)>0){
				Navigation::vertex vex;
				vex.coor.x = i*0.1+0.1;
				vex.coor.y = j*0.1+0.1;
				vex.id = vertex_list.size();
				vertex_list.push_back(vex);
			}
		}
	}
	////
	std::cout<<"vertex list:";
	for(int i=0;i<vertex_list.size();i++){
		std::cout<<"["<<vertex_list[i].coor.x<<","<<vertex_list[i].coor.y<<"],";
	}
	std::cout<<std::endl;
}

void Navigation::drive2Goal(Navigation::goal routePoint) {
		bool tmp = scan_analyzer.sector_IsSafe(2,0.15);
		if(!driveOn && ultraPass && tmp){
//			std::string tmp;
//			std::cin>>tmp;
			drive(0.3);
			driveOn=true;
			time_start = ros::Time::now().toSec();
		}
		else if(!ultraPass || !tmp){
			drive(0);
			route.clear();
			nextGoal = true;
		}

		if(goal_reached(routePoint) || (ros::Time::now().toSec()-time_start)>0.4){
			drive(0);
			driveOn=false;
			if(goal_reached(routePoint)&&route.size()>0)
				route.erase(route.begin());
			ROS_INFO("drive stopped.");
			//std::string temp;
			//std::cin>>temp;
		}
		//change_direct = false;

}

bool Navigation::goal_reached(Navigation::goal g) {

	return (fabs(robotPose.x - g.x) < drive_tol
			&& fabs(robotPose.y - g.y) < drive_tol); //different method because of angle
}




brain::MotorCommand Navigation::pubStop() {
	brain::MotorCommand cmd;
	cmd.name = "stop";
	cmd.r_effort = 0;
	cmd.l_effort = 0;
	return cmd;
}

void Navigation::rotate2AimAngle(double aimAngle) {
	double delta_alpha = (fmod(aimAngle - robotPose.theta + 3 * M_PI,
											2 * M_PI) - M_PI);
				//std::string temp;
				//std::cin>>temp;
				if(fabs(delta_alpha)<turn_threshold||
				fabs(delta_alpha)>1.95*M_PI){
					rotate(0);
					ROS_INFO("stop ratate");
					//std::string temp;
					//std::cin>>temp;
					rot_done = true;
					total_init_done = true;
				}
				else if(delta_alpha<0){
					if(ros::Time::now().toSec()-time_rotl>1){
						rotate(3);
						time_rotl = ros::Time::now().toSec();
					}
				}
				else if(delta_alpha>0){
					if(ros::Time::now().toSec()-time_rotr>1){
						rotate(-3);
						time_rotr = ros::Time::now().toSec();
					}
				}
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
	std::srand(time(NULL));
	ros::init(argc, argv, "Navigation");
	ros::NodeHandle n;
	Scan_analyzer sa;
	Navigation m = Navigation(n, sa);
//	ros::spinOnce();
//	m.scan_analyzer.refresh();
//	m.scan_analyzer.updateMap();
//	m.rotate(3.14);
//	ros::Time t_start = ros::Time::now();
//	while(ros::Time::now()-t_start < 5)

	while (ros::ok()) {
		ros::Rate loop_rate(30);
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
