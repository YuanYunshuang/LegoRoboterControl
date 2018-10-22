#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "geometry_msgs/PoseStamped.h"

// enum of chessboard types
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID }; 

// Structure for handling poses
struct Pose
{
	// Member variables
	cv::Point2d m_pos;
	double m_angle;
	ros::Time m_stamp;

	// Constructors
	Pose()
		: m_pos(0.0,0.0)
		, m_angle(0.0)
	{}

	Pose(const cv::Point2d& fr_point, const double fr_angle, const ros::Time& fr_stamp)
	{
		m_pos 	= fr_point;
		m_angle = fr_angle;
		m_stamp = fr_stamp;
	}

	// Generate formatted pose string
	std::string toString()
	{
		std::stringstream sstr;
		sstr	<< std::fixed << std::setfill(' ') << std::setprecision(3) << m_stamp.toSec() << "  " 
					<< std::setprecision(2)	<< std::setw(10) << m_pos.x << "  " << std::setw(10) << m_pos.y << "  " 
					<< std::setprecision(6) << std::setw(10) << m_angle << std::endl;
		return sstr.str();
	}
};

// The tracker class
class Tracker{

	// Member variables
	cv::Mat m_K;					// camera matrix
  cv::Mat m_distCoeffs; // distortion coefficients

	// containers for chessboard corners
  std::vector<cv::Point3f> m_corners3d;		
  std::vector<cv::Point2f> m_corners2d;

	// number of inner chessboard corners
	cv::Size m_boardSize;

	// container for the homography
	cv::Mat m_H;

	// output filestream for results
	std::ofstream m_trackOfstream; 

	// flag indicating tracker status
	bool m_hasHomography;

	// container for the pose parameters
	Pose m_pose;

	// 
  image_transport::ImageTransport m_imgTransport;

	// publisher and subscriber
  ros::Publisher  m_pub;
  image_transport::Subscriber m_sub;

public:

	// Constructor
	Tracker(ros::NodeHandle nh);

	// Methods
	int readCalibration(std::string& calibFilename);
  int openOutputFile(std::string filename);

	
	geometry_msgs::PoseStamped publishMessage();

	void calcChessboardCorners(cv::Size boardSize, float squareSize, Pattern patternType);
	void getPose( const sensor_msgs::ImageConstPtr& msg );

	
	
};
