
#include "lego_loam_utility.h"
#include <unordered_map>
#include <pcl/registration/ndt.h>
#include "pm_utility.h"
#include "tic_toc.hpp"
#include <string>


enum DataStreamStatus {
    YES = 1,
    NO = 2
};



class LidarMapLocalization{

private:
    ros::NodeHandle nh;

    // subs
    ros::Subscriber subSegmentedCloud;
    ros::Subscriber subFusionLocalization;

    // pub
    ros::Publisher pubMapLocalization;
    ros::Publisher pubGlobalMapMatch;
    ros::Publisher pubMapMatch;

    pcl::PointCloud<PointType>::Ptr localMapCloud =
            boost::make_shared<pcl::PointCloud<PointType>>();

    pcl::PointCloud<PointType>::Ptr globalMapCloud =
            boost::make_shared<pcl::PointCloud<PointType>>();

    std_msgs::Header current_header;
    pcl::PointCloud<PointType>::Ptr currentScan =
            boost::make_shared<pcl::PointCloud<PointType>>();
    nav_msgs::Odometry associated_fusion_loc;
    sensor_msgs::PointCloud2 associated_scan;


    std::mutex lidarMutex;
    std::mutex locMutex;
    std::queue<nav_msgs::Odometry> locBuf;
    std::queue<sensor_msgs::PointCloud2> scanBuf;

    std::string global_map_file;
    float init_x, init_y, init_z, init_roll, init_pitch, init_yaw;

    float max_delta_yaw_deg, max_delta_z, max_delta_xy, local_map_radius;
    float filter_resolution = 0.2;
    float localization_frequence;

    bool initialized = false;
    double system_begin_time = -1.0;
    double system_duration_time = -1.0;
    DataStreamStatus Lidar_Data_Stream_Status = DataStreamStatus::YES;
    DataStreamStatus Loc_Data_Stream_Status = DataStreamStatus::YES;
    double error_Lidar_Data_Stream_time = -1.0;
    double error_Loc_Data_Stream_time = -2.0;

    Eigen::Affine3f init_tf = Eigen::Affine3f::Identity();
    Eigen::Affine3f map_tf = Eigen::Affine3f::Identity();

public:
    LidarMapLocalization() {
      subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 100, &LidarMapLocalization::laserCloudHandler, this);
      subFusionLocalization = nh.subscribe<nav_msgs::Odometry>("/fusion_localization", 100, &LidarMapLocalization::laserLocalizationHandler, this);

      pubMapLocalization = nh.advertise<nav_msgs::Odometry> ("/map_localization", 20);
      pubGlobalMapMatch = nh.advertise<sensor_msgs::PointCloud2> ("/lidar_map_match_cloud", 20);
      pubMapMatch = nh.advertise<sensor_msgs::PointCloud2> ("/map_match_cloud", 20);

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

      for(auto &p : globalMapCloud->points) p.intensity = 0;

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

    bool CHECKDATASTREAM() {
      std::lock_guard<std::mutex> lock1(locMutex);
      std::lock_guard<std::mutex> lock2(lidarMutex);

      if(Lidar_Data_Stream_Status ==  DataStreamStatus::NO ||
         Loc_Data_Stream_Status ==  DataStreamStatus::NO) return false;

      double error_Data_Stream_time =
              std::max(error_Loc_Data_Stream_time, error_Lidar_Data_Stream_time);

      return int(error_Loc_Data_Stream_time * 1e5) == int(error_Lidar_Data_Stream_time * 1e5);
    }

    void pubMapLocalizationResult() {
      // publish localization
      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(map_tf, x, y, z, roll, pitch, yaw);

      // publish latest odometry
      nav_msgs::Odometry mapLocalizationResult;
      mapLocalizationResult.header = current_header;
      mapLocalizationResult.header.frame_id = "camera_init";
      mapLocalizationResult.pose.pose.position.x = x;
      mapLocalizationResult.pose.pose.position.y = y;
      mapLocalizationResult.pose.pose.position.z = z;
      mapLocalizationResult.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      pubMapLocalization.publish(mapLocalizationResult);

      // publish for visulization
      pcl::PointCloud<PointType> currentScanTf;
      pcl::PointCloud<PointType> transform_scan = *globalMapCloud;
      for(auto &p : localMapCloud->points) p.intensity = 210;
      transform_scan += *localMapCloud;
      for(auto &p : currentScan->points) p.intensity = 80;
      pcl::transformPointCloud(*currentScan, currentScanTf, map_tf);
      transform_scan += currentScanTf;

      sensor_msgs::PointCloud2 matchCloud;
      pcl::toROSMsg(transform_scan, matchCloud);
      matchCloud.header.stamp = current_header.stamp;
      matchCloud.header.frame_id = "camera_init";
      pubGlobalMapMatch.publish(matchCloud);

      // map match debug
      pcl::transformPointCloud(*currentScan, currentScanTf, init_tf);
      for(auto &p : currentScanTf.points) p.intensity = 160;
      transform_scan += currentScanTf;

      pcl::toROSMsg(transform_scan, matchCloud);
      matchCloud.header.stamp = current_header.stamp;
      matchCloud.header.frame_id = "camera_init";
      pubMapMatch.publish(matchCloud);
    }

    bool systemInitialize() {
      Eigen::Translation3f t(init_x, init_y, init_z);
      geometry_msgs::Quaternion quater = tf::createQuaternionMsgFromRollPitchYaw(init_roll, init_pitch, init_yaw);
      Eigen::Quaternionf q(quater.w, quater.x, quater.y, quater.z);
      init_tf = t * q;

      {
        std::lock_guard<std::mutex> lock1(lidarMutex);
        if(scanBuf.empty()) {
          ROS_WARN_ONCE("scanBuf empty, system initialize failed!");
          return false;
        }

        sensor_msgs::PointCloud2 cloudMsg = scanBuf.front();
        pcl::fromROSMsg(cloudMsg, *currentScan);

        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*currentScan, *currentScan, index);
        current_header = cloudMsg.header;

        while (!scanBuf.empty()) scanBuf.pop();
      }

      map_tf = mapLocalization(init_tf);

      system_begin_time = current_header.stamp.toSec();
      system_duration_time = 0.0;
      initialized = true;

      pubMapLocalizationResult();

      return true;
    }

    bool CachCurrentCloud() {
      {
        std::lock_guard<std::mutex> lock(locMutex);
        if(locBuf.empty()) {
          ROS_INFO("locBuf empty; Have no fusion data, return !");
          return false;
        }

        associated_fusion_loc = locBuf.back();

        /// clear history older loc message
        while(!locBuf.empty()) locBuf.pop();
      }

      current_header = associated_fusion_loc.header;

      {
        std::lock_guard<std::mutex> lock1(lidarMutex);
        if(scanBuf.empty()) {
          ROS_WARN("scanBuf empty, return !");
          return false;
        }

        associated_scan = scanBuf.front();
        scanBuf.pop();

        while(!scanBuf.empty() &&
              associated_scan.header.stamp.toSec() < (current_header.stamp.toSec() - 0.006)) {
          associated_scan = scanBuf.front();
          scanBuf.pop();
        }

        double time_gap = fabs(associated_scan.header.stamp.toSec() -
                               associated_fusion_loc.header.stamp.toSec());

        if(time_gap > 0.006) {
          ROS_INFO("\n Have no valid associated fusion loc and scan data ! time_gap: %f \n", time_gap);
          return false;
        }

        pcl::fromROSMsg(associated_scan, *currentScan);

        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*currentScan, *currentScan, index);
      }

      return true;
    }

    bool CachAssociatedFusionLoc() {
      std::lock_guard<std::mutex> lock(locMutex);
      if(locBuf.empty()) {
        ROS_INFO("locBuf empty; Have no fusion data !");
        return false;
      }

      associated_fusion_loc = locBuf.front();
      locBuf.pop();

      while(!locBuf.empty() &&
            associated_fusion_loc.header.stamp.toSec() < (current_header.stamp.toSec() - 0.006)) {
        associated_fusion_loc = locBuf.front();
        locBuf.pop();
      }

      double time_gap = fabs(associated_fusion_loc.header.stamp.toSec() - current_header.stamp.toSec());
      if(time_gap < 0.005) {
        return true;

      } else {
        ROS_INFO("\n Have no valid associated fusion data ! time_gap: %f \n", time_gap);
        return false;
      }
    }

    void run() {
      if(!initialized) {
        if(systemInitialize()) {
          ROS_INFO("System initialize successfully !");
        } else {
          return;
        }
      }

      if(!CachCurrentCloud()) return;

      init_tf = odomToAffine(associated_fusion_loc);

      TicToc t_map;
      map_tf = mapLocalization(init_tf);

      std::cout << "map localization finish, " << t_map.toc() << std::endl;

      pubMapLocalizationResult();
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
      ndt.setMaximumIterations(100);

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

      std::vector<int> index;
      pcl::removeNaNFromPointCloud(*map, *map, index);
      pcl::removeNaNFromPointCloud(*scan, *scan, index);

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

      trans_matrix = pointMatcherICP(map, scan, init_tf);

      return trans_matrix;
    }

    bool CheckMapMatchTF(const Eigen::Affine3f &delta_tf) {
      if(system_duration_time < 20) return true;

      float dx, dy, dz, droll, dpitch, dyaw;
      pcl::getTranslationAndEulerAngles(delta_tf, dx, dy, dz, droll, dpitch, dyaw);

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

    Eigen::Affine3f mapLocalization(const Eigen::Affine3f &init_tf) {
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
//      if(CheckMapMatchTF(match_tf.inverse() * ndt_tf)) {
        match_tf = ndt_tf;
//      }

//      if(CheckMapMatchTF(match_tf.inverse() * icp_tf)) {
//        match_tf = icp_tf;
//      }

      return match_tf;
    }

    void laserLocalizationHandler(const nav_msgs::Odometry::ConstPtr& fusionLocMsg) {
      static double last_time = -100.0;
      static nav_msgs::Odometry last_odometry;
      std::lock_guard<std::mutex> lock(locMutex);

      if(initialized) {
        if(fusionLocMsg->header.stamp.toSec() < last_time ||
           fabs(fusionLocMsg->header.stamp.toSec() - last_time) > 0.3) {
          ROS_WARN("Occur fusion loc data stream timestamp problem ! last_time: %f, current_time: %f ",
                   last_time, fusionLocMsg->header.stamp.toSec());
          Loc_Data_Stream_Status = DataStreamStatus::NO;
          error_Loc_Data_Stream_time = fusionLocMsg->header.stamp.toSec();
          last_time = fusionLocMsg->header.stamp.toSec();
          return;
        }
      }

      Loc_Data_Stream_Status = DataStreamStatus::YES;
      locBuf.push(*fusionLocMsg);
      last_time = fusionLocMsg->header.stamp.toSec();
      last_odometry = *fusionLocMsg;
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
      static double last_time = -100.0;
      std::lock_guard<std::mutex> lock(lidarMutex);

      if(initialized) {
        if (laserCloudMsg->header.stamp.toSec() < last_time ||
            fabs(laserCloudMsg->header.stamp.toSec() - last_time) > 0.3) {
          ROS_WARN("Occur lidar data stream timestamp problem ! last_time: %f, current_time: %f ",
                   last_time, laserCloudMsg->header.stamp.toSec());
          Lidar_Data_Stream_Status = DataStreamStatus::NO;
          error_Lidar_Data_Stream_time = laserCloudMsg->header.stamp.toSec();
          last_time = laserCloudMsg->header.stamp.toSec();
          return;
        }
      }

      Lidar_Data_Stream_Status = DataStreamStatus::YES;
      scanBuf.push(*laserCloudMsg);
      last_time = laserCloudMsg->header.stamp.toSec();
    }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "mapLocalization");

  LidarMapLocalization mapLocalization;

  ROS_INFO("\033[1;32m---->\033[0m Map Localization Started.");

  ros::Rate rate(2);

  while (ros::ok()) {
    ros::spinOnce();

    mapLocalization.run();

    rate.sleep();
  }

  return 0;
}
