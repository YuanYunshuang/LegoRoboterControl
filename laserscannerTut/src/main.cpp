/*
 * main.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: adminuser
 */

#include <iostream>
#include "Parkour.h"
#include"Robot.h"
#include <math.h>
#include <vector>
#include "Sensor.h"
#include "ScanAnalyzer.h"
using namespace std;
using namespace cv;
/*
 * move getting an input an moves x,y for 10
 * 	    left/right/up/down by a/d/w/s
 * 		forward by space
 * 		rotate left/right by q/e
 */
void move(char input, double& x, double& a, double& y, double s) {
	switch (input) {
	case ' ':
		x += cos(a) * s;
		y += sin(a) * s;
		break;
	case 'd':
		x += s;
		break;
	case 'a':
		x -= s;
		break;
	case 'w':
		y -= s;
		break;
	case 's':
		y += s;
		break;
	case 'q':
		a += M_PI / 200 * s;
		a = fmod(a + 3 * M_PI, 2 * M_PI) - M_PI;
		break;
	case 'e':
		a -= M_PI / 200 * s;
		a = fmod(a + 3 * M_PI, 2 * M_PI) - M_PI;
		break;
	default:
		break;
	}
}

cv::Mat drawPoles(const Parkour p, const vector<ScanAnalyzer::pole> pole_map,
		const vector<ScanAnalyzer::pole> global_poles,
		const vector<ScanAnalyzer::pole> poles,
		const vector<ScanAnalyzer::pole> global_poles_corr) {
	cv::Mat global_pole_image(p.vis.size(), p.vis.type(),
			Scalar(255, 255, 255));
	for (int i = 0; i < pole_map.size(); i++) {
		circle(global_pole_image,
				cv::Point2d(pole_map[i].x,
						global_pole_image.rows - pole_map[i].y),
				pole_map[i].r + 2, cv::Scalar(255, 0, 0), -1);
	}
	for (int i = 0; i < global_poles.size(); i++) {
		circle(global_pole_image,
				cv::Point2d(global_poles[i].x,
						global_pole_image.rows - global_poles[i].y),
				global_poles[i].r + 1, cv::Scalar(0, 0, 255), -1);
	}
	for (int i = 0; i < poles.size(); i++) {
		circle(global_pole_image,
				cv::Point2d(poles[i].x / 2 + global_pole_image.cols / 2,
						global_pole_image.rows - poles[i].y / 2
								- global_pole_image.rows / 2), poles[i].r,
				cv::Scalar(200, 200, 200), 1);
		circle(global_pole_image,
				cv::Point2d(global_poles_corr[i].x,
						global_pole_image.rows - global_poles_corr[i].y),
				global_poles_corr[i].r, cv::Scalar(0, 255, 0), -1);
	}
	resize(global_pole_image, global_pole_image, Size(400, 400));
	return global_pole_image;
}

void drawScan(const vector<double>& dist, const Robot& r, const Pose& odom,
		cv::Mat& measurement_img, cv::Mat& simulation, Mat& map) {
	for (int i = 0; i < dist.size(); i++) {
		double beamAngle = fmod(
				r.pose.phi
						- r.sensors[0].alpha / 2+i*r.sensors[0].beta + 3 * M_PI,
				M_PI * 2) - M_PI;
		double beamAngle2 = fmod(
				odom.phi
						- r.sensors[0].alpha / 2+i*r.sensors[0].beta + 3 * M_PI,
				M_PI * 2) - M_PI;
		double x = r.pose.x + cos(beamAngle) * dist[i];
		double y = r.pose.y + sin(beamAngle) * dist[i];
		double x2 = odom.x + cos(beamAngle2) * dist[i];
		double y2 = odom.y + sin(beamAngle2) * dist[i];
		line(measurement_img, Point2d(i, measurement_img.rows - 1),
				Point2d(i, measurement_img.rows - 1 - dist[i]),
				Scalar(255, 0, 0));
		Scalar color(255, 0, 0);
		if (i >dist.size()/2-5&&i<dist.size()/2+5) {
			color = Scalar(0, 255, 0);
		}
		line(simulation, Point2d(r.pose.x, simulation.rows - r.pose.y),
				Point2d(x, simulation.rows - y), color);
		vector<cv::Point2d> actualScan;
		circle(map, Point2d(x2, map.rows - y2), 1, cv::Scalar(0, 0, 0),
				-1);

	}
	resize(measurement_img, measurement_img, Size(400, 400));
	resize(simulation, simulation, Size(400, 400));
	resize(map, map, Size(400, 400));
}

int main() {
	namedWindow("measurement", WINDOW_AUTOSIZE);
	moveWindow("measurement", 800, 0);
	namedWindow("GAME", WINDOW_AUTOSIZE);
	namedWindow("MAP", WINDOW_AUTOSIZE);
	moveWindow("MAP", 0, 800);
	namedWindow("global Poles", WINDOW_AUTOSIZE);
	moveWindow("global Poles", 800, 800);
	Parkour p;
	Robot r;
	double beta = 0.01;
	r.sensors.push_back(Sensor(M_PI, beta, 0, 0, 0, 1000));
	r.pose.phi = 0;
	r.pose.x = 50;
	r.pose.y = 200;
	Mat map(p.vis.size(), p.vis.type(), Scalar(255, 255, 255));
	ScanAnalyzer sa(beta, 5);
	double speed = 10;
	Pose odomPose = r.sensors[0].getPose(r.pose);
	static vector<ScanAnalyzer::pole> pole_map;
	while (true) {
		vector<double> dist, der;
		r.measurement(0, p, dist);
		p.build();
		cv::Mat simulation, measurementImg((int) r.sensors[0].range,
				dist.size(), CV_8UC3, Scalar(255, 255, 255));
		p.vis.copyTo(simulation);
		der = sa.compute_derivative(dist);
		vector<ScanAnalyzer::pole> poles = sa.getUpdate(dist, der);
		cout << poles.size() << " Poles found" << endl;
		//Task build transformation and transform to global coordinates
		//cv::Mat sensorTrafo =(cv::Mat_<double>(2, 3) << 1,0,r.pose.x,0,1,r.pose.y);
		cv::Mat sensorTrafo =(cv::Mat_<double>(2, 3) << cos(odomPose.phi),-sin(odomPose.phi),odomPose.x,sin(odomPose.phi),cos(odomPose.phi),odomPose.y);
		vector<ScanAnalyzer::pole> global_poles = sa.transformPoles(
				sensorTrafo, poles);

//Calculate correction
		cv::Mat cor = (cv::Mat_<double>(2, 3) << 1, 0, 0, 0, 1, 0);
		vector<ScanAnalyzer::pole> global_poles_corr = sa.adjustUpdate(
				global_poles, pole_map, cor);
		if (pole_map.empty()) {
			cout<<"empty"<<endl;
			pole_map = global_poles_corr;
		} else {
//Merge map update
			sa.mergeMap(global_poles_corr, pole_map);
			cout<<"no empty"<<endl;
		}

		//Correct Odom
		cout<<"draw Poles"<<endl;
		cv::Mat global_pole_image = drawPoles(p, pole_map, global_poles, poles,
				global_poles_corr);
		cv::imshow("global Poles", global_pole_image);
		cout<<"draw odom"<<endl;
		drawScan(dist, r, odomPose,measurementImg, simulation, map);
		imshow("measurement", measurementImg);
		imshow("GAME", simulation);
		imshow("MAP", map);
		char input = cv::waitKey(0);
		move(input, r.pose.x, r.pose.phi, r.pose.y, speed);
		move(input, odomPose.x, odomPose.phi, odomPose.y, speed);
		odomPose.x += (static_cast<double>(rand()) / RAND_MAX - 0.5) * speed
				/ 50;
		odomPose.y += (static_cast<double>(rand()) / RAND_MAX - 0.5) * speed
				/ 50;
		odomPose.phi = fmod(
				odomPose.phi
						+ (static_cast<double>(rand()) / RAND_MAX - 0.5)
								* 0.1+3 * M_PI, 2 * M_PI) - M_PI;

	}
}

