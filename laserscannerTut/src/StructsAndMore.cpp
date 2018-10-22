/*
 * StructsAndMore.cpp
 *
 *  Created on: Dec 1, 2017
 *      Author: adminuser
 */
#include "StructsAndMore.h"



std::vector<std::string> ikg::split(std::string line,char delimiter){
std::vector<std::string> tmp;
int pos = line.find(",");
		while (pos != -1){
		pos = line.find(",");
		tmp.push_back(line.substr(0, pos));
		line = line.substr(pos + 1);
		}
		return tmp;
}

