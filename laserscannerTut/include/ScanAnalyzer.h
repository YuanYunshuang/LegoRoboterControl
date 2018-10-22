/*
 * ScannAnalyzer.h
 *
 *  Created on: Dec 5, 2017
 *      Author: adminuser
 */

#ifndef SCANANALYZER_H_
#define SCANANALYZER_H_
#include <opencv2/opencv.hpp>
class ScanAnalyzer {
public:
	double angle_increment,cylinder_offset,tol;
	struct pole {
		pole(double x=0, double y=0, double r=0):x(x),y(y),r(r){};
			double x, y, r;
		};
	std::vector<pole> adjustUpdate(std::vector<pole> map_update,std::vector<pole> map,cv::Mat new_transformation);
	std::vector<double> compute_derivative(const std::vector<double> act_scan);
	std::vector<pole> getUpdate(std::vector<double> scan,std::vector<double> der);
	pole compute_cartesian(double mean_ray, double mean_depth, double rays);
	ScanAnalyzer(double angle,double cylinder);
	void mergeMap(const std::vector<pole> measurement,std::vector<pole> &map );
	virtual ~ScanAnalyzer();
	std::vector<pole> transformPoles(cv::Mat transformation,
				const std::vector<pole> mapUpdate);

private:

};

#endif /* SCANNANALYZER_H_ */
