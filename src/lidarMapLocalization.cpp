
#include "lego_loam_utility.h"
#include <unordered_map>
#include <pcl/registration/ndt.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Parametrizable.h>
#include <string>


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

DP convertPCL2PM(const pcl::PointCloud<PointType> &cloud) {
  std::size_t point_number = cloud.size();

  DP pm_points;
  pm_points.features.resize(4, point_number);

  for (std::size_t i = 0; i < point_number; i++) {
    pm_points.features(0, i) = cloud.points[i].x;
    pm_points.features(1, i) = cloud.points[i].y;
    pm_points.features(2, i) = cloud.points[i].z;
    pm_points.features(3, i) = 1.0;
  }

//  pm_points.featureLabels.push_back(DP::Label("x", 1));
//  pm_points.featureLabels.push_back(DP::Label("y", 1));
//  pm_points.featureLabels.push_back(DP::Label("z", 1));
//  pm_points.featureLabels.push_back(DP::Label("pad", 1));
//
//  pm_points.descriptors.resize(1, point_number);
//  pm_points.descriptorLabels.push_back(DP::Label("intensity", 1));
//
//  for (std::size_t i = 0; i < point_number; i++) {
//    pm_points.features(0, i) = cloud.points[i].x;
//    pm_points.features(1, i) = cloud.points[i].y;
//    pm_points.features(2, i) = cloud.points[i].z;
//    pm_points.features(3, i) = 1.0;
//    pm_points.descriptors(0, i) = cloud.points[i].intensity;
//  }
  return pm_points;
}





class LidarMapLocalization{

private:
    ros::NodeHandle nh;

    // subs
    ros::Subscriber subSegmentedCloud;
    ros::Subscriber subFusionLocalization;

    // pub
    ros::Publisher pubMapLocalization;
    ros::Publisher pubGlobalMapMatch;
    ros::Publisher pubInitMapMatch;

    pcl::PointCloud<PointType>::Ptr localMapCloud =
            boost::make_shared<pcl::PointCloud<PointType>>();

    pcl::PointCloud<PointType>::Ptr globalMapCloud =
            boost::make_shared<pcl::PointCloud<PointType>>();

    pcl::PointCloud<PointType>::Ptr currentScan =
            boost::make_shared<pcl::PointCloud<PointType>>();

    nav_msgs::Odometry mapLocalization;

    std::mutex scantx;
    std::mutex localizationtx;
    std::queue<nav_msgs::Odometry> localizations;
//    std::queue<sensor_msgs::PointCloud2> scanBuf;

    std::string global_map_file;
    float init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;

    float max_delta_yaw_deg, max_delta_z, max_delta_xy, local_map_radius;
    float filter_resolution = 0.2;
    float localization_frequence;

    bool initialized = false;
    double system_begin_time = -1.0;
    double system_duration_time = -1.0;

public:
    LidarMapLocalization() {
      subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 100, &LidarMapLocalization::laserCloudHandler, this);
      subFusionLocalization = nh.subscribe<nav_msgs::Odometry>("/fusion_localization", 100, &LidarMapLocalization::laserLocalizationHandler, this);

      pubMapLocalization = nh.advertise<nav_msgs::Odometry> ("/map_localization", 20);
      pubGlobalMapMatch = nh.advertise<sensor_msgs::PointCloud2> ("/lidar_map_match_cloud", 20);
      pubInitMapMatch = nh.advertise<sensor_msgs::PointCloud2> ("/init_map_match_cloud", 20);

      LoadParam();

      loadGlobalMapCloud();
    }

    void LoadParam() {
      nh.param<float>("init_x", init_x, 20.0);
      nh.param<float>("init_y", init_y, -6.0);
      nh.param<float>("init_z", init_z, 0.0);

      nh.param<float>("init_roll", init_roll, 0.0);
      nh.param<float>("init_pitch", init_pitch, 0.0);
      nh.param<float>("init_yaw", init_yaw, 2.0);

      nh.param<std::string>("globalMapFile", global_map_file, "/home/xl/Desktop/ht_office/opt/all_feature_map.pcd");

      nh.param<float>("max_delta_yaw", max_delta_yaw_deg, 10.0);
      nh.param<float>("max_delta_z", max_delta_z, 0.40);
      nh.param<float>("max_delta_xy", max_delta_xy, 0.9);

      nh.param<float>("local_map_radius", local_map_radius, 13);
      nh.param<float>("filter_resolution", filter_resolution, 0.2);
      nh.param<float>("localization_frequence", localization_frequence, 0.5);

      ROS_INFO("Hello! Finished to load Init pose parameters: %f, %f, %f, %f, %f, %f, %s",
               init_x, init_y, init_z, init_roll, init_pitch, init_yaw, global_map_file.c_str());

      ROS_INFO("Hello! Finished to load match parameters: %f, %f, %f, %f, %f",
               max_delta_yaw_deg, max_delta_z, max_delta_xy, local_map_radius, filter_resolution);
    }

    void loadGlobalMapCloud() {
      if (pcl::io::loadPCDFile(global_map_file, *globalMapCloud) < 0) {
        ROS_ERROR("Ooh, Failed load global map cloud! Check global_map_file: ", global_map_file.c_str());
      }
      std::cout << global_map_file << std::endl;

      std::vector<int> index;
      pcl::removeNaNFromPointCloud(*globalMapCloud, *globalMapCloud, index);

      pcl::VoxelGrid<pcl::PointXYZI> sor;
      sor.setInputCloud(globalMapCloud);
      sor.setLeafSize(filter_resolution, filter_resolution, filter_resolution);
      sor.filter(*globalMapCloud);

      ROS_INFO("Hello! Finished to load global map cloud ! %d ", globalMapCloud->size());
    }

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

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
      static double last_time = -100.0;

      std::lock_guard<std::mutex> lock1(scantx);
//      scanBuf.push(*laser_Msg);
//      if(scanBuf.size() < 3) return;
//      sensor_msgs::PointCloud2 laserCloudMsg = scanBuf.front();
//      scanBuf.pop();

      double current_timestamp = laserCloudMsg->header.stamp.toSec();
      if(fabs(current_timestamp - last_time) < localization_frequence) return;
      last_time = current_timestamp;
      pcl::fromROSMsg(*laserCloudMsg, *currentScan);

      static Eigen::Affine3f init_tf = Eigen::Affine3f::Identity();

      std::vector<int> index;
      pcl::removeNaNFromPointCloud(*currentScan, *currentScan, index);


      if(!initialized && localizations.empty()) {
        Eigen::Translation3f t(init_x, init_y, init_z);
        geometry_msgs::Quaternion quater = tf::createQuaternionMsgFromRollPitchYaw(init_roll, init_pitch, init_yaw);
        Eigen::Quaternionf q(quater.w, quater.x, quater.y, quater.z);
        init_tf = t * q;
        initialized = true;
        system_begin_time = last_time;
        system_duration_time = 0;
        ROS_INFO("-------------------  Now,  Finished to init tf --------------------");

      } else {
        system_duration_time = laserCloudMsg->header.stamp.toSec() - system_begin_time;
        std::lock_guard<std::mutex> lock1(localizationtx);
        if(!localizations.empty()) {
          double current_fusionloc_timestamp = localizations.back().header.stamp.toSec();
          nav_msgs::Odometry nearest_odo = localizations.front();
          localizations.pop();

          while (nearest_odo.header.stamp.toSec() < current_timestamp && (!localizations.empty())) {
            nearest_odo = localizations.front();
            localizations.pop();
          }

          if(fabs(nearest_odo.header.stamp.toSec() - current_timestamp) < 0.3) {
            init_tf = odomToAffine(nearest_odo);
            ROS_INFO("nearest_odo to init_tf, message time delta: %f ", current_fusionloc_timestamp - current_timestamp);
          }

        } else {
          ROS_INFO("Have no fusion localization data !");
        }
      }

      Eigen::Affine3f tf = coreLocalization(init_tf);

      // publish localization
      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(tf, x, y, z, roll, pitch, yaw);

      // publish latest odometry
      mapLocalization.header = laserCloudMsg->header;
      mapLocalization.header.frame_id = "camera_init";
      mapLocalization.pose.pose.position.x = x;
      mapLocalization.pose.pose.position.y = y;
      mapLocalization.pose.pose.position.z = z;
      mapLocalization.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      pubMapLocalization.publish(mapLocalization);

      // publish for visulization
      {
        pcl::PointCloud<PointType> transform_scan = *globalMapCloud;
        for(auto &p : localMapCloud->points) p.intensity = 100;
//      transform_scan += *localMapCloud;
        for(auto &p : currentScan->points) p.intensity = 210;
        pcl::transformPointCloud(*currentScan, *currentScan, tf);
        transform_scan += *currentScan;

        sensor_msgs::PointCloud2 matchCloud;
        pcl::toROSMsg(transform_scan, matchCloud);
        matchCloud.header.stamp = laserCloudMsg->header.stamp;
        matchCloud.header.frame_id = "camera_init";
        pubGlobalMapMatch.publish(matchCloud);

        transform_scan.clear();
        for(auto &p : localMapCloud->points) p.intensity = 255;
        transform_scan = *localMapCloud;
        pcl::transformPointCloud(*currentScan, *currentScan, init_tf * tf.inverse());
        transform_scan += *currentScan;
        pcl::toROSMsg(transform_scan, matchCloud);
        matchCloud.header.stamp = laserCloudMsg->header.stamp;
        matchCloud.header.frame_id = "camera_init";
        pubInitMapMatch.publish(matchCloud);
      }


      init_tf = tf;
    }

    Eigen::Affine3f ndtLoalization(const pcl::PointCloud<PointType>::Ptr &map,
                                   const pcl::PointCloud<PointType>::Ptr &scan,
                                   const Eigen::Affine3f &init_tf) {

      // Initializing Normal Distributions Transform (NDT).
      pcl::NormalDistributionsTransform<PointType, PointType> ndt;

      // Setting scale dependent NDT parameters
      // Setting minimum transformation difference for termination condition.
      ndt.setTransformationEpsilon(0.01);

      // Setting maximum step size for More-Thuente line search.
      ndt.setStepSize(0.1);

      // Setting Resolution of NDT grid structure (VoxelGridCovariance).
      ndt.setResolution(1);

      // Setting max number of registration iterations.
      ndt.setMaximumIterations(35);

      // Setting point cloud to be aligned.
      ndt.setInputSource(scan);
      // Setting point cloud to be aligned to.
      ndt.setInputTarget(map);

      // Calculating required rigid transform to align the input cloud to the target
      // cloud.
      pcl::PointCloud<PointType>::Ptr output_cloud(
              new pcl::PointCloud<PointType>());
      ndt.align(*output_cloud, init_tf.matrix());

      if (!ndt.hasConverged()) {
        ROS_INFO("has no Converged");
        return init_tf;
      } else {
//        ROS_INFO("has Converged");
        Eigen::Matrix4f tf = ndt.getFinalTransformation();
        return Eigen::Affine3f(tf);
      }
    }

    Eigen::Affine3f pointMatcherICP(const pcl::PointCloud<PointType>::Ptr &map,
                                    const pcl::PointCloud<PointType>::Ptr &scan,
                                    const Eigen::Affine3f &init_tf) {
      Eigen::Matrix4f trans_matrix;
      PM::ICP icp;
      icp.setDefault();

      DP map_p = convertPCL2PM(*map);
      DP scan_p = convertPCL2PM(*scan);

      trans_matrix = icp(scan_p, map_p, init_tf.matrix());
      return Eigen::Affine3f(trans_matrix);
    }

    Eigen::Affine3f pclICP(const pcl::PointCloud<PointType>::Ptr &map,
                           const pcl::PointCloud<PointType>::Ptr &scan,
                           const Eigen::Affine3f &init_tf) {
      static pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
      icp.setMaximumIterations(100);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setRANSACIterations(0);

      // Align clouds
      icp.setInputSource(scan);
      icp.setInputTarget(map);
      pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
      icp.align(*unused_result);

      if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return init_tf;

      Eigen::Affine3f correctionLidarFrame;
      correctionLidarFrame = icp.getFinalTransformation();

      return correctionLidarFrame;
    }

    Eigen::Affine3f icpLoalization(const pcl::PointCloud<PointType>::Ptr &map,
                                   const pcl::PointCloud<PointType>::Ptr &scan,
                                   const Eigen::Affine3f &init_tf) {
      Eigen::Affine3f trans_matrix;

//      trans_matrix = pointMatcherICP(map, scan, init_tf);

      trans_matrix = pclICP(map, scan, init_tf);

      return trans_matrix;
    }

    bool CheckMapMatchTF(const Eigen::Affine3f &delta_tf) {
      if(system_duration_time < 20) return true;

      float dx, dy, dz, droll, dpitch, dyaw;
      pcl::getTranslationAndEulerAngles(delta_tf, dx, dy, dz, droll, dpitch, dyaw);
      std::cout << "------------------------------------" << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

      if(dyaw > DEG2RAD(max_delta_yaw_deg)) {
        ROS_INFO("large delta yaw, beyound 10 deg. RAD2DEG(dyaw): ", RAD2DEG(dyaw));
        return false;
      }

      if(dz > max_delta_z) {
        ROS_INFO("large delta z, beyound max_delta_z. dz: %f, max_delta_z: %f", dz, max_delta_z);
        return false;
      }

      double dxy = sqrt(pow(dx, 2) + pow(dy, 2));
      if(dxy > max_delta_xy) {
        ROS_INFO("large delta xy, beyound max_delta_xy. dxy: %f, max_delta_xy: %f", dxy, max_delta_xy);
        return false;
      }

      return true;
    }

    int searchLocalMap(const Eigen::Vector3f &centroid) {
      *localMapCloud = *globalMapCloud;

      localMapCloud->points.erase(
              remove_if(localMapCloud->points.begin(), localMapCloud->points.end(),
                        [&](const PointType& p) -> bool {
                            Eigen::Vector3f p_g(p.x, p.y, p.z);
                            if((p_g - centroid).norm() > local_map_radius)
                              return true;
                            else
                              return false;
                        }), localMapCloud->points.end());

      return localMapCloud->size();
    }

    Eigen::Affine3f coreLocalization(const Eigen::Affine3f &init_tf) {
      Eigen::Vector3f centroid{init_tf(0,3), init_tf(1,3), init_tf(2,3)};
      int local_map_number = searchLocalMap(centroid);

      Eigen::Affine3f match_tf = init_tf;
      Eigen::Affine3f ndt_tf, icp_tf;

      if(local_map_number > 10000) {
        ndt_tf = ndtLoalization(localMapCloud, currentScan, init_tf);
//        icp_tf = icpLoalization(localMapCloud, currentScan, init_tf);
      } else {
        ndt_tf = ndtLoalization(globalMapCloud, currentScan, init_tf);
//        icp_tf = icpLoalization(localMapCloud, currentScan, init_tf);
      }

      /// map localization desicion strategy
      if(CheckMapMatchTF(match_tf.inverse() * ndt_tf)) {
        match_tf = ndt_tf;
      }

//      if(CheckMapMatchTF(match_tf.inverse() * icp_tf)) {
//        match_tf = icp_tf;
//      }

      return match_tf;
    }

    void laserLocalizationHandler(const nav_msgs::Odometry::ConstPtr& laserLocalizationMsg) {
      std::lock_guard<std::mutex> lock1(localizationtx);
      localizations.push(*laserLocalizationMsg);
    }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "mapLocalization");

  LidarMapLocalization mapLocalization;

  ROS_INFO("\033[1;32m---->\033[0m Map Localization Started.");

  ros::spin();

  return 0;
}
