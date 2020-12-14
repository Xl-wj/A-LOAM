#pragma once

#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include "lego_loam_utility.h"

Eigen::Affine3f odomToAffine(nav_msgs::Odometry odom) {
  double x, y, z, roll, pitch, yaw;
  x = odom.pose.pose.position.x;
  y = odom.pose.pose.position.y;
  z = odom.pose.pose.position.z;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
  orientation.normalized();
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}
