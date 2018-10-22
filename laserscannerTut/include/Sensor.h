/*
 * Sensor.h
 *
 *  Created on: Nov 29, 2017
 *      Author: adminuser
 */

#ifndef SENSOR_H_
#define SENSOR_H_
#include <memory>
#include <vector>
struct Pose{
	double x,y,phi;
	Pose():x(0),phi(0),y(0){};
	Pose(double x,double y,double phi):x(x),y(y),phi(phi){};
};

class Sensor {
	int measurement;
public:
	Pose pose;
	double alpha, beta,range;
	Sensor(double alpha, double beta,double x, double y, double phi, double range);
	Sensor();
	virtual ~Sensor();
int getMeasurement();
Pose getPose(const Pose r);
//	bool getMeasurment(double mesaurment, const Parkour p);
};

#endif /* SENSOR_H_ */
