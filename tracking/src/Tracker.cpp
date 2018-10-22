#include "Tracker.h"
using namespace cv;  
using namespace std;

// *********** utility methods ***********

// compute coordinates of chessboard corners in object space
void Tracker::calcChessboardCorners(cv::Size boardSize, float squareSize, Pattern patternType)
{
	// clear vectors
	m_corners3d.clear();
	m_corners2d.clear();

	// set member variables to argument values
	m_boardSize = boardSize;
	int nPoints = boardSize.width*boardSize.height;

	// compute corners for different types of patterns
	switch(patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
			for( int j = 0; j < boardSize.width; j++ )
			{
				m_corners3d.push_back(cv::Point3f(float(i*squareSize), float(j*squareSize), 0.0f)); 
				m_corners2d.push_back(cv::Point2f(float(i*squareSize), float(j*squareSize)));
			}
			break;

	case ASYMMETRIC_CIRCLES_GRID:
		for( int i = 0; i < boardSize.height; i++ )
			for( int j = 0; j < boardSize.width; j++ )
			{
				m_corners3d.push_back(cv::Point3f(float((2*i + j % 2)*squareSize), float(j*squareSize), 0.0f));
				m_corners2d.push_back(cv::Point2f(float((2*i + j % 2)*squareSize), float(j*squareSize)));
			}
			break;

	default:
		CV_Error(CV_StsBadArg, "Unknown pattern type\n");
	}

	// save coordinates
	std::ofstream fout("/home/adminuser/tmp/chessboard.txt");
	for(int i=0; i<nPoints; i++)
	{
		fout<<m_corners3d[i].x<<"\t"<<m_corners3d[i].y<<"\t"<<0.0f<<std::endl;
	}

}

// read an OpenCV calibration file
int Tracker::readCalibration(std::string& calibFilename)
{
	// read calibration data
	cv::FileStorage fs_calib_in( calibFilename, cv::FileStorage::READ );
	if(!fs_calib_in.isOpened()){
		std::cout << "\nCould not read calibration parameters from " << calibFilename << std::endl;
		return 1;
	}

	fs_calib_in["intrinsics"]    >> m_K;
	fs_calib_in["c_dist_coeffs"] >> m_distCoeffs;
	return 0;
}

// prepare output file to save computed poses
int Tracker::openOutputFile(std::string filename)
{
	// make file to hold all estimated robot positions
	m_trackOfstream.open(filename.c_str());
	if(!m_trackOfstream){
		std::cout << "\nCould not open output file " << filename << std::endl;
		return 1;
	}

	m_trackOfstream << " timestamp [s] " << std::setw(5) << "" << "x [mm]" 
									<< std::setw(6) << "" << "y [mm]" << std::setw(5) << "" << "heading [rad]" << std::endl;
	return 0;
}


// *********** core functionality ***********

// constructor
Tracker::Tracker(ros::NodeHandle nh)
    : m_K(3,3,CV_64F)
		, m_hasHomography(false)
		,	m_boardSize(0,0)
		, m_imgTransport(nh)
{
		// init the tracker, cv::Size = [width,height]
		//calcChessboardCorners(cv::Size(9,6) 35.0f, CHESSBOARD); // adapt number of chessboard corners and their size
		calcChessboardCorners(cv::Size(5,4), 0.034f, CHESSBOARD); // adapt number of chessboard corners and their size
		openOutputFile("/home/adminuser/tmp/track.txt");

		// start communicating
		m_pub = nh.advertise<geometry_msgs::PoseStamped>("tracking/pose", 10);
		m_sub = m_imgTransport.subscribe("camera/image",1,&Tracker::getPose,this);
}


// publish results as PoseStamped message
geometry_msgs::PoseStamped Tracker::publishMessage()
{
	geometry_msgs::PoseStamped msg;
  // TODO
	msg.pose.position.x=m_pose.m_pos.x;
	msg.pose.position.y=m_pose.m_pos.y;
	msg.pose.position.z= 0 ;
	msg.header.stamp=m_pose.m_stamp;
	msg.pose.orientation.x=m_pose.m_angle;
	msg.pose.orientation.y=0;
	msg.pose.orientation.z=m_pose.m_angle/sqrt(1+m_pose.m_angle*m_pose.m_angle);
	msg.pose.orientation.w=1/sqrt(m_pose.m_angle*m_pose.m_angle);
  //done
	return msg;
}



// main tracking method 
void Tracker::getPose( const sensor_msgs::ImageConstPtr& msg )
{

	// Get image and timestamp  时间戳
	ros::Time t   = msg->header.stamp;
	cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

  // TODO   
  //https://docs.opencv.org/2.4/doc/tutorials/features2d/trackingmotion/corner_subpixeles/corner_subpixeles.html 
   vector<Point2f> corners;  // 定义二维点
   vector<Point2f> newObjectPoints;

bool found=findChessboardCorners(image, m_boardSize, corners,CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE);  //bool: true or falsh
   
    //cvFindCornerSubPix(&srcIp, image_points_buf, nowNumber, cvSize(9,6), cvSize(-1,-1),  
                                //cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));   //亚像素subpixel	
if (found)
{
   drawChessboardCorners(image, m_boardSize, corners, found);   //绘制棋盘角点。found
	
   if (m_hasHomography == false)
	{
  	 m_H=findHomography(corners, m_corners2d);
	 m_hasHomography = true;
	} 
	if (m_hasHomography)
	{
 	//棋盘原点：width-1;夹角=atan2(delta_y,delta_x)
 	perspectiveTransform(corners, newObjectPoints, m_H);
 	m_pose.m_pos = newObjectPoints[0];
 	m_pose.m_angle = atan2(newObjectPoints[m_boardSize.width -1].x-newObjectPoints[0].x , newObjectPoints[m_boardSize.width -1].y-newObjectPoints[0].y); 
 	m_pose.m_stamp = t;

	}
	m_trackOfstream << m_pose.toString();
	m_pub.publish(publishMessage());
}
  // done  
	cv::imshow("Tracking Image View (PTZ)", image);
	cv::waitKey(1);


}




