/*
 * Parkour.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: adminuser
 */

#include "Parkour.h"
#include<fstream>
#include"StructsAndMore.h"
using namespace cv;
using namespace std;
Parkour::Parkour() {
build();
}

Parkour::~Parkour() {
}

void Parkour::build(){
	ifstream ifstr("parkour2.txt");
		string line;
		while (getline(ifstr, line)) {
			vector<string> input = ikg::split(line, ',');
			if (input[0] == "Box"||input[0]=="Wall") {
				rectangle(vis,
						Rect(stoi(input[1]), (int) stoi(input[2]), stoi(input[3]),
								stoi(input[4])), Scalar(0, 0, 0), -1);
			} else if (input[0] == "Circle") {
				cout<<input[1]<<endl;
				circle(vis, Point2d(stoi(input[1]), stoi(input[2])), stoi(input[3]),
						Scalar(0, 0, 0), -1);
			} else if (input[0] == "Parkour") {
				vis = cv::Mat(stoi(input[1]), stoi(input[2]), CV_8UC3,
						cv::Scalar(255, 255, 255));
			}
		}
}
