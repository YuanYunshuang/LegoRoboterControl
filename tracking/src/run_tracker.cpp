#include "Tracker.h"


int main(int argc, char **argv)
{
  ros::init(argc,argv,"run_tracker");
  ros::NodeHandle nh;
  Tracker tr(nh);
  ros::spin();
  return 0;
}
