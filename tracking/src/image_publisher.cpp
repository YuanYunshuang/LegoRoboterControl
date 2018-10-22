#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class ImagePublisher{

	cv::Mat m_image;
	cv::VideoCapture m_cap;

	image_transport::ImageTransport m_imageTransport;
	image_transport::Publisher m_pub;
	sensor_msgs::ImagePtr m_msg;

public:
	ImagePublisher(ros::NodeHandle nh)
		//: m_cap("http://root:tet@192.168.2.102/mjpg/video.mjpg")     //!!!!!!!!
		: m_cap("http://192.168.2.101/video.cgi?.mjpg")
		, m_imageTransport(nh)
	{
		ROS_INFO("image_publisher is running.");
		m_pub = m_imageTransport.advertise("camera/image", 1);
		publish();
	}

private:
	void publish()
	{
		ros::Time t;
		ros::Rate loop_rate(30);     //!!!!!!!!!!!!!!!!!!

		//cv::VideoWriter vOut("/home/adminuser/tmp/a4.avi",CV_FOURCC('F','M','P','4'),25,cv::Size(704,576));

		while (ros::ok()) {

			if(!m_cap.read(m_image)){
				ROS_ERROR("video stream not working.");
				break;
			}      //correkt!!!!!!!!!!!!!!!!!!!!
			//m_image=cv::imread("/home/adminuser/tmp/chessImage.png");
			t = ros::Time::now();
	
			// Debug: Output of current image
		  cv::imshow("image_publisher",m_image);
		  cv::waitKey(1);
			//cv::imwrite("/home/adminuser/img.jpg",m_image);
			//vOut << m_image;

		  m_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_image).toImageMsg();
			m_msg->header.stamp = t;

		  m_pub.publish(m_msg);
		  ros::spinOnce();
		  loop_rate.sleep();
		}
	}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

	ImagePublisher ip(nh);
}


