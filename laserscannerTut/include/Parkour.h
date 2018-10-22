/*
 * Parkour.h
 *
 *  Created on: Oct 27, 2017
 *      Author: adminuser
 */

#ifndef PARKOUR_H_
#define PARKOUR_H_
#include <opencv2/opencv.hpp>
class Parkour {
public:

	cv::Mat vis;
	Parkour();
	virtual ~Parkour();
	void build();
};

#endif /* PARKOUR_H_ */
