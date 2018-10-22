/*
 * ScanAnalyzer.cpp
 *
 *  Created on: Dec 5, 2017
 *      Author: adminuser
 */

#include "ScanAnalyzer.h"
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <math.h>
using namespace std;

ScanAnalyzer::ScanAnalyzer(double angle, double cylinder) {
	angle_increment = angle;
	cylinder_offset = cylinder;
	tol = 50;
}

void ScanAnalyzer::mergeMap(const std::vector<pole> measurement,
		std::vector<pole> &map) {
	for (int i = 0; i < measurement.size(); i++) {
		bool add = true;
		for (int j = 0; j < map.size(); j++) {
			if (sqrt(
					pow(map[j].x - measurement[i].x, 2)
							+ pow(map[j].y - measurement[i].y, 2)) < tol) {
				add = false;
			}
		}
		if (add) {
			map.push_back(measurement[i]);
		}
	}
}

std::vector<ScanAnalyzer::pole> ScanAnalyzer::transformPoles(
		cv::Mat transformation,
		const std::vector<ScanAnalyzer::pole> mapUpdate) {
	std::vector<ScanAnalyzer::pole> newUpdate;
	double c, s, tx, ty;
	//todo replace
	pole p;

	c = transformation.at<double>(0, 0);
	s = transformation.at<double>(1, 0);
	tx = transformation.at<double>(0, 2);
	ty = transformation.at<double>(1, 2);

	for (int i = 0; i < mapUpdate.size(); i++) {
		p.x = c * mapUpdate[i].x - s * mapUpdate[i].y + tx;
		p.y = s * mapUpdate[i].x + c * mapUpdate[i].y + ty;
		p.r = mapUpdate[i].r;
		newUpdate.push_back(p);
	}
	//newUpdate=mapUpdate;
	return newUpdate;
}

std::vector<ScanAnalyzer::pole> ScanAnalyzer::adjustUpdate(
		std::vector<ScanAnalyzer::pole> map_update,
		std::vector<ScanAnalyzer::pole> map, cv::Mat new_transformation) {
	std::vector<pair<ScanAnalyzer::pole, ScanAnalyzer::pole>> merged_poles;
//find correspondence pairs and add to merge poles use tolerance "tol"
	for (int i = 0; i < map.size(); i++) {
		for (int j = 0; j < map_update.size(); j++) {
			if (sqrt(
					pow(map[i].x - map_update[j].x, 2)
							+ pow(map[i].y - map_update[j].y, 2)) < tol) {
				std::pair<ScanAnalyzer::pole, ScanAnalyzer::pole> pole_pair(
						map.at(i), map_update.at(j));
				merged_poles.push_back(pole_pair);
				cout << "i" << i << endl;
				cout <<"("<< pole_pair.first.x <<","<<pole_pair.first.y <<")" << endl;

				cout <<"("<< pole_pair.second.x <<","<<pole_pair.second.y <<")" << endl;
			}
		}
	}

	if (merged_poles.size() == 0) {
		return map_update;
	}
	cout << "MERGE " << merged_poles.size() << endl;

//return mapUpdate if no pairs found
	cv::Mat p1 = cv::Mat(2, merged_poles.size(), CV_64F, 0.0), p2 = cv::Mat(2,
			merged_poles.size(), CV_64F, 0.0);
	double mean_x1 = 0, mean_x2 = 0, mean_y1 = 0, mean_y2 = 0;
//calculate center of gravity
	/*
	 * x1 y1 old poles already in the map merged_poles.first
	 * x2 y2 new poles need to be updated merged_poles.second
	 */
	for (int i = 0; i < merged_poles.size(); i++) {
		// Calculate the sum of the old poles
		mean_x1 = mean_x1 + merged_poles[i].first.x;
		mean_y1 = mean_y1 + merged_poles[i].first.y;
		mean_x2 = mean_x2 + merged_poles[i].second.x;
		mean_y2 = mean_y2 + merged_poles[i].second.y;
	}

	// Calculate mean
	mean_x1 = mean_x1 / merged_poles.size();
	mean_x2 = mean_x2 / merged_poles.size();
	mean_y1 = mean_y1 / merged_poles.size();
	mean_y2 = mean_y2 / merged_poles.size();

	//reduce center of gravity
	/*
	 * p1 old poles reduced
	 * p2 new poles reduced
	 */
	for (int i = 0; i < merged_poles.size(); i++) {
		p1.at<double>(0, i) = merged_poles[i].first.x - mean_x1;
		p1.at<double>(1, i) = merged_poles[i].first.y - mean_y1;
		p2.at<double>(0, i) = merged_poles[i].second.x - mean_x2;
		p2.at<double>(1, i) = merged_poles[i].second.y - mean_y2;
	}

	//calculate cos(a), sin(a)  by least square alignment
//	------------------------------------------------------------------------------------------
	cv::Mat A = cv::Mat(2 * merged_poles.size(), 2, CV_64F, 0.0);
	cv::Mat l = cv::Mat(2 * merged_poles.size(), 1, CV_64F, 0.0);
	cv::Mat x_cs = cv::Mat(2, CV_64F, 0.0);

	for (int i = 0; i < merged_poles.size(); i++) {
		A.at<double>(i * 2, 0) = p2.at<double>(0, i);
		A.at<double>(i * 2, 1) = -p2.at<double>(1, i);

		A.at<double>(i * 2 + 1, 0) = p2.at<double>(1, i);
		A.at<double>(i * 2 + 1, 1) = p2.at<double>(0, i);

		l.at<double>(i * 2) = p1.at<double>(0, i);
		l.at<double>(i * 2 + 1) = p1.at<double>(1, i);
	}

	// (A^TA)^-1 A^T l
	cv::Mat AT, ATA, invATA, Qxx;

	transpose(A, AT);
	ATA = AT * A;
	cv::invert(ATA, invATA,cv::DECOMP_SVD);
	Qxx = invATA * AT;
	x_cs = Qxx * l;

//calculates a and build R
	double alpha = 0;
	alpha = atan2(x_cs.at<double>(1,0),x_cs.at<double>(0,0));
	cout << "alpha" << alpha << endl;
	cv::Mat R =
			(cv::Mat_<double>(2, 2) << cos(alpha), -sin(alpha), sin(alpha), cos(alpha));
	//(cv::Mat_<double>(2, 2) << x_cs.at<double>(0,0),-x_cs.at<double>(1,0),x_cs.at<double>(1,0),x_cs.at<double>(0,0));
			//(cv::Mat_<double>(2, 2) << 1, 0, 0, 1);
	//calculates translation

//	cv::Mat t_z = (cv::Mat_<double>(1, 2) << 0, 0);
//	cv::Mat t = (cv::Mat_<double>(1, 2) << 0, 0);
//	cv::Mat mp1 = cv::Mat(2,merged_poles.size(),CV_64F, 0.0);
//	cv::Mat mp2 = cv::Mat(2,merged_poles.size(),CV_64F, 0.0);
//
//	for (int i = 0; i < merged_poles.size(); i++) {
//			mp1.at<double>(0, i) = merged_poles[i].first.x ;
//			mp1.at<double>(1, i) = merged_poles[i].first.y ;
//			mp2.at<double>(0, i) = merged_poles[i].second.x;
//			mp2.at<double>(1, i) = merged_poles[i].second.y;
//		}

	//-------------------------------------------------------------------------------------
//
//	t_z = mp1-R*mp2;
//
//	for (int i = 0; i < merged_poles.size(); i++)
//	{
//		t.at<double>(0,0) = t.at<double>(0,0) + t_z.at<double>(0, i);
//		t.at<double>(0,1) = t.at<double>(0,1) + t_z.at<double>(1, i);
//	}
//
//	t = t/merged_poles.size();

	double tx = mean_x1-cos(alpha) * mean_x2 + sin(alpha) * mean_y2;
	double ty = mean_y1-sin(alpha) * mean_x2 - cos(alpha) * mean_y2;

	cv::Mat t = (cv::Mat_<double>(1, 2) << tx, ty);
	//-------------------------------------------------------------------------------------

	cout << "R" << R << endl << "T" << t << endl;
//transform each map Update with R and t and update new_transform
	new_transformation = (cv::Mat_<double>(2, 3) << R.at<double>(0, 0), R.at<
			double>(0, 1), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(
			1, 1), t.at<double>(1, 0));
	pole p;
	std::vector<ScanAnalyzer::pole> map_new;
	for (int i = 0; i < map_update.size(); i++) {
		p.x = cos(alpha) * map_update[i].x - sin(alpha) * map_update[i].y + t.at<double>(0,0);
		p.y = sin(alpha) * map_update[i].x + cos(alpha) * map_update[i].y + t.at<double>(0,1);
		p.r = map_update[i].r;
		map_new.push_back(p);
	}
	//cout << "mU" << map_update << endl;
	return map_new;
}
std::vector<double> ScanAnalyzer::compute_derivative(
		const std::vector<double> actScan) {
	std::vector<double> der;
	der.push_back(0.0);
	for (int x = 1; x < actScan.size() - 1; x++) {
		if (!isnan(actScan[x])) {
			der.push_back((actScan[x + 1] - actScan[x - 1]) * 0.5);

		} else {
			der.push_back(0.0);
		}
	}
	return der;

}
std::vector<ScanAnalyzer::pole> ScanAnalyzer::getUpdate(
		std::vector<double> scan, std::vector<double> der) {
	double shr = 5;
	double min_dist = 1;
	bool on_cylinder = false;
	double sum_ray = 0, sum_depth = 0, rays = 0, mean_ray = 0, mean_depth = 0;
	std::vector<ScanAnalyzer::pole> map_update;
	for (int i = 1; i < scan.size(); i++) {
		if (std::isnan(scan[i] || std::isinf(scan[i])) || scan[i] < min_dist)
			continue;
		if (der[i] < -shr) {
			on_cylinder = true;
			sum_ray = 0, rays = 0, sum_depth = 0;
		}
		if (on_cylinder) {
			sum_ray += i;
			sum_depth += scan[i];
			rays++;
		}
		if (der[i] > shr && on_cylinder) {
			mean_ray = sum_ray / rays;
			mean_depth = sum_depth / rays;
			on_cylinder = false;
			pole new_pole = compute_cartesian(mean_ray, mean_depth, rays);
			map_update.push_back(new_pole);

		} else {
		}

	}

	return map_update;
}
ScanAnalyzer::pole ScanAnalyzer::compute_cartesian(double mean_ray,
		double mean_depth, double rays) {
	double x = 0, y = 0, r = 0, depth = 0, theta = 0;
	depth = mean_depth + cylinder_offset;
	theta = (mean_ray * angle_increment);
	x = depth * sin(theta);
	y = -depth * cos(theta);
	r = depth * (rays / 2) * angle_increment;
	ScanAnalyzer::pole newPole = { x, y, r };
	return newPole;
}

ScanAnalyzer::~ScanAnalyzer() {
// TODO Auto-generated destructor stub
}

