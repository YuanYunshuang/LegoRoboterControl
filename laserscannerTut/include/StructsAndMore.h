/*
 * StructsAndMore.h

 *
 *  Created on: Dec 1, 2017
 *      Author: Busch
 */
#include <vector>
#include <string>
struct Pose{
	double x,y,phi;
	Pose():x(0),phi(0),y(0){};
	Pose(double x,double y,double phi):x(x),y(y),phi(phi){};
};
namespace ikg{
std::vector<std::string> split(std::string line,char delimiter);

}
