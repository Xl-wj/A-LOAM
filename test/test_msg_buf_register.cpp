
#include "ros_msg_buf_register.h"
#include <nav_msgs/Path.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "RosMsgBufRegister");

  ROS_INFO("\033[1;32m---->\033[0m Ros Msg Buf Register Started.");

  ros::NodeHandle nh;

  MsgBufRegister<sensor_msgs::PointCloud2> scan_buf(nh, "/laser_cloud_sharp", 1000, "scan_buf");
  MsgBufRegister<nav_msgs::Odometry> odo_buf(nh, "/odometry/imu_incremental", 1000, "odo_buf");

  ros::Rate rate(1);

  while (ros::ok()) {
    ros::spinOnce();

    std::cout << "scan size: " << scan_buf.size() << std::endl;
    std::cout << "odo  size: " << odo_buf.size() << std::endl;
    scan_buf.clear();
    odo_buf.clear();

    rate.sleep();
  }

  return 0;
}
