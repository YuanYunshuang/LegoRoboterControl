/*
 * Sensor.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: adminuser
 */

#include "Sensor.h"
#include <math.h>
#include<assert.h>
#include<iostream>
using namespace std;
Sensor::Sensor(double alpha, double beta, double x, double y, double phi,
		double range) :	alpha(alpha), beta(beta), range(range) {
	pose=Pose(x,y,phi);
	measurement = floor(alpha / beta);
	assert(measurement > 1);

}
int Sensor::getMeasurement() {
	return measurement;
}
Pose Sensor::getPose(const Pose r) {
	Pose p;
	p.x = r.x + cos(r.phi) * pose.x - sin(r.phi) * pose.y;
	p.y = r.y + sin(r.phi) * pose.x + cos(r.phi) * pose.y;
	p.phi = fmod(r.phi + pose.phi + 3 * M_PI, 2 * M_PI) - M_PI;
	return p;
}
Sensor::~Sensor() {
//	delete [] dist;
}

