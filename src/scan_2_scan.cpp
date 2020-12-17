#include "ros_msg_buf_register.h"
#include "scan_2_scan_odo.h"
#include "lego_loam_utility.h"
#include <unordered_map>
#include <pcl/registration/ndt.h>
#include "pm_utility.h"
#include "tic_toc.hpp"
#include <string>
#include <iomanip>
#include <nav_msgs/Path.h>

#include <glog/logging.h>
#include "tic_toc.hpp"
#include "hdmap_poly.h"
#include "tf_common.h"

class SubMessageCallback {
public:
    SubMessageCallback(ros::NodeHandle &nh) {
      cornerSharpBuf = std::make_shared<MsgBufRegister<sensor_msgs::PointCloud2>>(nh, "/laser_cloud_sharp", 100);
      cornerLessSharpBuf = std::make_shared<MsgBufRegister<sensor_msgs::PointCloud2>>(nh, "/laser_cloud_less_sharp", 100);
      surfFlatBuf = std::make_shared<MsgBufRegister<sensor_msgs::PointCloud2>>(nh, "/laser_cloud_flat", 100);
      surfLessFlatBuf = std::make_shared<MsgBufRegister<sensor_msgs::PointCloud2>>(nh, "/laser_cloud_less_flat", 100);
      fullPointsBuf = std::make_shared<MsgBufRegister<sensor_msgs::PointCloud2>>(nh, "/velodyne_cloud_2", 100);
      fusionLocalization = std::make_shared<MsgBufRegister<nav_msgs::Odometry>>(nh, "/fusion_localization", 100);
    }

    bool ScanMsgReady() {

      if (cornerSharpBuf->empty() || cornerLessSharpBuf->empty() ||
          surfFlatBuf->empty() || surfLessFlatBuf->empty() || fullPointsBuf->empty()) return false;

      double timeCornerPointsSharp = cornerSharpBuf->frontTime();
      double timeCornerPointsLessSharp = cornerLessSharpBuf->frontTime();
      double timeSurfPointsFlat = surfFlatBuf->frontTime();
      double timeSurfPointsLessFlat = surfLessFlatBuf->frontTime();
      double timeLaserCloudFullRes = fullPointsBuf->frontTime();

      scanFrame.timestamp = timeLaserCloudFullRes;

      double current_time = std::max(timeCornerPointsSharp, timeCornerPointsLessSharp);
      current_time = std::max(current_time, timeSurfPointsFlat);
      current_time = std::max(current_time, timeSurfPointsLessFlat);

      cornerSharpBuf->clear(current_time);
      cornerLessSharpBuf->clear(current_time);
      surfFlatBuf->clear(current_time);
      surfLessFlatBuf->clear(current_time);
      fullPointsBuf->clear(current_time);

      if (timeCornerPointsSharp != scanFrame.timestamp || timeCornerPointsLessSharp != scanFrame.timestamp ||
          timeSurfPointsFlat != scanFrame.timestamp || timeSurfPointsLessFlat != scanFrame.timestamp) {

        printf(" unsync messeage(scan-to-scan):"
               " time timeLaserCloudFullRes %f"
               " timeCornerPointsSharp %f"
               " timeCornerPointsLessSharp %f"
               " timeSurfPointsFlat %f"
               " timeSurfPointsLessFlat %f \n",
               timeLaserCloudFullRes,
               timeCornerPointsSharp,
               timeCornerPointsLessSharp,
               timeSurfPointsFlat,
               timeSurfPointsLessFlat);
        return false;
      }

      scanFrame.cornerPointsSharp.clear();
      scanFrame.cornerPointsLessSharp.clear();
      scanFrame.surfPointsFlat.clear();
      scanFrame.surfPointsLessFlat.clear();

      pcl::fromROSMsg(cornerSharpBuf->frontMsg(), (scanFrame.cornerPointsSharp));
      pcl::fromROSMsg(cornerLessSharpBuf->frontMsg(), (scanFrame.cornerPointsLessSharp));
      pcl::fromROSMsg(surfFlatBuf->frontMsg(), (scanFrame.surfPointsFlat));
      pcl::fromROSMsg(surfLessFlatBuf->frontMsg(), (scanFrame.surfPointsLessFlat));
      pcl::fromROSMsg(fullPointsBuf->frontMsg(), (scanFrame.fullPoints));

      cornerSharpBuf->clear();
      cornerLessSharpBuf->clear();
      surfFlatBuf->clear();
      surfLessFlatBuf->clear();
      fullPointsBuf->clear();

      return true;
    }

    MatcherFrame::Ptr GetMatcherFrame() const {
      return std::make_shared<MatcherFrame>(scanFrame);
    }

    bool LocalizationMsgReady(double timestamp) {
      if(fusionLocalization->empty()) return false;

      if(fabs(fusionLocalization->endTime() - timestamp) < 1e-3) {
        fusionLocalization->clear(timestamp);
        pose = odomToAffine(fusionLocalization->frontMsg());
        return true;
      }
    }

    Eigen::Affine3f GetFusionPose() const { return pose; }

private:
    MatcherFrame scanFrame;
    Eigen::Affine3f pose;

    MsgBufRegister<sensor_msgs::PointCloud2>::Ptr cornerSharpBuf, cornerLessSharpBuf;
    MsgBufRegister<sensor_msgs::PointCloud2>::Ptr surfFlatBuf, surfLessFlatBuf, fullPointsBuf;
    MsgBufRegister<nav_msgs::Odometry>::Ptr fusionLocalization;
};

class PubMessageCallback {
public:
    PubMessageCallback(ros::NodeHandle &nh) {
      pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
      pubLaserPath = nh.advertise<nav_msgs::Path>("/scan_2_scan_path", 100);
      pubLaserCloudMatch = nh.advertise<sensor_msgs::PointCloud2>("/scan_2_scan_cloud_match", 100);
      pubRoadBoundaryPath = nh.advertise<nav_msgs::Path>("/road_boundary", 100);
    }

    void PublishOdom(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, double timestamp) {
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.frame_id = "/camera_init";
      laserOdometry.child_frame_id = "/laser_odom";
      laserOdometry.header.stamp = ros::Time().fromSec(timestamp);
      laserOdometry.pose.pose.orientation.x = q.x();
      laserOdometry.pose.pose.orientation.y = q.y();
      laserOdometry.pose.pose.orientation.z = q.z();
      laserOdometry.pose.pose.orientation.w = q.w();
      laserOdometry.pose.pose.position.x = t.x();
      laserOdometry.pose.pose.position.y = t.y();
      laserOdometry.pose.pose.position.z = t.z();
      pubLaserOdometry.publish(laserOdometry);

      // publish path
      static nav_msgs::Path odo_path;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time().fromSec(timestamp);
      pose_stamped.header.frame_id = "camera_init";
      pose_stamped.pose = laserOdometry.pose.pose;
      odo_path.poses.push_back(pose_stamped);

      if (pubLaserPath.getNumSubscribers() != 0) {
        odo_path.header.stamp = ros::Time().fromSec(timestamp);
        odo_path.header.frame_id = "camera_init";
        pubLaserPath.publish(odo_path);
      }
    }

    void PublishMatch(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const MatcherFrame::Ptr &frame) {
      static MatcherFrame last_frame;
      if(last_frame.timestamp < 10) {
        last_frame = *frame;
        return;
      }
      Eigen::Affine3d tf = Eigen::Translation3d(t) * q;
      pcl::PointCloud<PointType> viewPoints;
      pcl::transformPointCloud(frame->fullPoints, viewPoints, tf.matrix());
      viewPoints += last_frame.fullPoints;

      sensor_msgs::PointCloud2 odoCloud;
      pcl::toROSMsg(viewPoints, odoCloud);
      odoCloud.header.stamp = ros::Time().fromSec(frame->timestamp);
      odoCloud.header.frame_id = "camera_init";
      pubLaserCloudMatch.publish(odoCloud);

      last_frame = *frame;
    }

    void PublishRoadBoundary(const XYZs &road_boundary, double timestamp) {
      // publish path
      static nav_msgs::Path roadBoundaryPath;
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(timestamp);
        pose_stamped.header.frame_id = "camera_init";

        for(const auto &point : road_boundary) {
          pose_stamped.pose.position.x = point.x;
          pose_stamped.pose.position.y = point.y;
          pose_stamped.pose.position.z = point.z;
          roadBoundaryPath.poses.push_back(pose_stamped);
        }

        roadBoundaryPath.header.stamp = ros::Time().fromSec(timestamp);
        roadBoundaryPath.header.frame_id = "camera_init";
      }
      pubRoadBoundaryPath.publish(roadBoundaryPath);
    }

private:
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath, pubLaserCloudMatch, pubRoadBoundaryPath;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "Scan2ScanOdomEstimation");
  ROS_INFO("\033[1;32m---->\033[0m Scan 2 Scan Odometry Estimation Started.");

  // hd polygen geometry
  std::string hdmap_polygen_file = "";
  LOG(INFO) << "Load hdmap polygen geometry !";
  XYZs hdmap_points = LoadHdMapRoadBoundary(hdmap_polygen_file);

  HDMapTaskFrame hdmap_polygen;
  hdmap_polygen.init(hdmap_points);


  // icp estimater
  Scan2ScanOdomEstimation scan_2_scan_odom;
  scan_2_scan_odom.SetHDMapTaskFrame(
          std::make_shared<HDMapTaskFrame>(hdmap_polygen));

  // ros message
  ros::NodeHandle nh;
  SubMessageCallback sub_msg_callback(nh);
  PubMessageCallback pub_msg_callback(nh);

  Eigen::Vector3d init_pose;
  Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
  Eigen::Vector3d t_w_curr(0, 0, 0);
  double last_scan_timestamp = -1.0;
  double delta_time_gap = -1.0;
  Eigen::Vector3d last_velocity;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    if(sub_msg_callback.ScanMsgReady()) {
      TicToc tt;
      MatcherFrame::Ptr frame = sub_msg_callback.GetMatcherFrame();

      if(last_scan_timestamp > 0) {
        delta_time_gap = frame->timestamp - last_scan_timestamp;
        init_pose = last_velocity * delta_time_gap;
        scan_2_scan_odom.SetInitPose(init_pose);

        if(delta_time_gap > 0.3) {
          LOG(WARNING) << "delta_time_gap > 0.3s, system stop " << delta_time_gap
                       << " s, and predict position ! init_pose: "
                       << init_pose[0] << " " << init_pose[1] << " " << init_pose[2];
        }

        if(sub_msg_callback.LocalizationMsgReady(last_scan_timestamp)) {
          Eigen::Affine3f last_pose = sub_msg_callback.GetFusionPose();
//          scan_2_scan_odom.FilterLastFrame(last_pose);

        }
      }

      scan_2_scan_odom.Run(sub_msg_callback.GetMatcherFrame());
      auto result =scan_2_scan_odom.GetResult();

      {
        Eigen::Quaterniond q_last_curr(result.q_w, result.q_x, result.q_y, result.q_z);
        Eigen::Vector3d t_last_curr(result.t_x, result.t_y, result.t_z);

        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;

        pub_msg_callback.PublishOdom(q_w_curr, t_w_curr, sub_msg_callback.GetMatcherFrame()->timestamp);
        pub_msg_callback.PublishMatch(q_last_curr, t_last_curr, sub_msg_callback.GetMatcherFrame());
        pub_msg_callback.PublishRoadBoundary(hdmap_points, sub_msg_callback.GetMatcherFrame()->timestamp);

        // keep last velocity
        if(delta_time_gap > 0.0) {
          last_velocity = (q_w_curr * t_last_curr) / delta_time_gap;
        }
        last_scan_timestamp = frame->timestamp;
      }

      if(tt.toc_milli(false) > 30) {
        LOG(INFO) << "Feature icp finished, " << tt.toc();
      }
    }

    rate.sleep();
  }

  return 0;
}
