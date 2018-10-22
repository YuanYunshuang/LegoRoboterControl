/*
 * Robot.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: adminuser
 */

#include "Robot.h"
#include<vector>
using namespace cv;
using namespace std;
Robot::Robot() {
	pose.x = 0;
	pose.y = 0;
	pose.phi = 0;
	transparentColor = Scalar(255, 0, 0, 0);
	shape = cv::Mat(15, 20, CV_8UC4, transparentColor);
	std::vector<Point2i> points;
	points.push_back(Point2i(0, 0));
	points.push_back(Point2i(shape.cols - 1, shape.rows / 2));
	points.push_back(Point2i(0, shape.rows - 1));
	line(shape, points[0], points[1], Scalar(0, 255, 0, 1), 2);
	line(shape, points[0], points[2], Scalar(0, 255, 0, 1), 2);
	line(shape, points[1], points[2], Scalar(0, 255, 0, 1), 2);
	fillConvexPoly(shape,               //Image to be drawn on
			points,                 //C-Style array of points
			Scalar(0, 255, 0, 1),  //Color , BGR form
			CV_AA,             // connectedness, 4 or 8
			0);
}
void Robot::measurement(int i, const Parkour p, vector<double> &measurement) {
	Pose sp = sensors[i].getPose(pose);
	for (int j = 0; j < sensors[i].getMeasurement(); j++) {
		double x = 0, y = 0, dist = 0;
		double a = fmod(
				sp.phi - sensors[i].alpha / 2 + j * sensors[i].beta + 3 * M_PI,
				M_PI * 2) - M_PI;
		for (int l = 1; l < sensors[i].range*10; l++) {
			x = sp.x + cos(a) * ((double) l/10.0);
			y = sp.y + sin(a) * ((double) l/10.0);
			try {
				if(x>p.vis.cols||y>p.vis.rows)throw "error";
				Vec3b color = p.vis.at<Vec3b>(p.vis.rows-(int) y, (int) x);

				dist = l/10.0;///+static_cast<double>((rand()%100))/50.0;
				if (color[0] != 255 && color[1] != 255 && color[2] != 255) {
					break;
				}
			} catch (...) {
//				cout << "Error" << endl;
				dist=sensors[0].range;
				break;
			}
		}
		measurement.push_back((double) dist);
	}
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

