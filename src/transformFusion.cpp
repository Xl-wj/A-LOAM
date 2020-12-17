
#include "lego_loam_utility.h"
#include <unordered_map>
#include <nav_msgs/Path.h>

class TransformFusion{

private:
    ros::NodeHandle nh;

    // pub
    ros::Publisher pubFusionLocalization;
    ros::Publisher pubLocalizationPath;
    // subs
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subMapLocalization;

    nav_msgs::Odometry laserOdometry;
    nav_msgs::Odometry mapLocalization;
    nav_msgs::Odometry fusionLocalization;

    std::mutex maptx;
    std::mutex odotx;

    std::queue<nav_msgs::Odometry> history_odoms_queue;
    std::unordered_map<size_t, nav_msgs::Odometry> history_odoms_map;

    double lastest_map_localization_time = -1.0;
    nav_msgs::Odometry lastest_map_localization_odom;
    Eigen::Affine3f lastest_map_tf;

public:
    TransformFusion() {
      pubFusionLocalization = nh.advertise<nav_msgs::Odometry> ("/fusion_localization", 100);
      pubLocalizationPath = nh.advertise<nav_msgs::Path>("/localization_path", 100);

      subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, &TransformFusion::laserOdometryHandler, this);
      subMapLocalization = nh.subscribe<nav_msgs::Odometry>("/map_localization", 100, &TransformFusion::mapLocalizationHandler, this);
    }

    size_t hashKeyOdometry(const nav_msgs::Odometry& laserOdometry) {
      return size_t(laserOdometry.header.stamp.toSec() * 1e7);
    }

    Eigen::Affine3f odomToAffine(nav_msgs::Odometry odom) {
      double x, y, z, roll, pitch, yaw;
      x = odom.pose.pose.position.x;
      y = odom.pose.pose.position.y;
      z = odom.pose.pose.position.z;
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {

      std::lock_guard<std::mutex> lock1(odotx);
      std::lock_guard<std::mutex> lock2(maptx);

      history_odoms_queue.push(*laserOdometry);
      auto itm = history_odoms_map.find(hashKeyOdometry(*laserOdometry));
      if (itm != history_odoms_map.end()) {
        ROS_WARN("Ooh, Error odometry hash value ! ");
        return;
      }
      history_odoms_map[hashKeyOdometry(*laserOdometry)] = *laserOdometry;

      // check system work
      if(lastest_map_localization_time < 0) return;

      Eigen::Affine3f current_odometry_tf = odomToAffine(*laserOdometry);
      if(history_odoms_map.find(hashKeyOdometry(lastest_map_localization_odom)) == history_odoms_map.end()) {
        ROS_WARN("Ooh, Failed to find lastest map localization ! ");
        return;
      }
      nav_msgs::Odometry history_lastest_map_odo = history_odoms_map[hashKeyOdometry(lastest_map_localization_odom)];
      Eigen::Affine3f history_lastest_map_tf = odomToAffine(history_lastest_map_odo);
      current_odometry_tf = lastest_map_tf * history_lastest_map_tf.inverse() * current_odometry_tf;

      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(current_odometry_tf, x, y, z, roll, pitch, yaw);

      // publish latest odometry
      fusionLocalization.header = laserOdometry->header;
      fusionLocalization.header.frame_id = "camera_init";
      fusionLocalization.pose.pose.position.x = x;
      fusionLocalization.pose.pose.position.y = y;
      fusionLocalization.pose.pose.position.z = z;
      fusionLocalization.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

      fusionLocalization.pose.covariance.at(0) = roll;
      fusionLocalization.pose.covariance.at(1) = pitch;
      fusionLocalization.pose.covariance.at(2) = yaw;
      pubFusionLocalization.publish(fusionLocalization);

      // publish path
      static nav_msgs::Path localizationPath;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = fusionLocalization.header.stamp;
      pose_stamped.header.frame_id = "camera_init";
      pose_stamped.pose = fusionLocalization.pose.pose;
      localizationPath.poses.push_back(pose_stamped);

      if (pubLocalizationPath.getNumSubscribers() != 0) {
        localizationPath.header.stamp = fusionLocalization.header.stamp;
        localizationPath.header.frame_id = "camera_init";
        pubLocalizationPath.publish(localizationPath);
      }
    }

    void mapLocalizationHandler(const nav_msgs::Odometry::ConstPtr& mapLocalization) {
      std::lock_guard<std::mutex> lock(maptx);
      auto itm = history_odoms_map.find(hashKeyOdometry(*mapLocalization));
      if (lastest_map_localization_time > 0 && (itm == history_odoms_map.end())) {
        ROS_WARN("Ooh, Map localization don't match any history odometrys! ", itm->first);
      }

      lastest_map_localization_odom = *mapLocalization;
      lastest_map_localization_time = mapLocalization->header.stamp.toSec();
      lastest_map_tf = odomToAffine(*mapLocalization);
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "transformFusion");

  TransformFusion TFusion;

  ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

  ros::spin();

  return 0;
}
