#include "pm_utility.h"
#include "lego_loam_utility.h"
#include <unordered_map>
#include <nav_msgs/Path.h>
#include "tic_toc.hpp"

class LidarPmOdometry {

private:
    ros::NodeHandle nh;

    // subs
    ros::Subscriber subSegmentedCloud;

    // pub
    ros::Publisher pubPmOdometry;

    bool init = false;
    Eigen::Affine3f last_odometry_matrix;
    DP currentScan, lastScan;
    PM::ICP icp;

public:
    LidarPmOdometry() {
        subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 100, &LidarPmOdometry::laserCloudHandler, this);
        pubPmOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

        icp.setDefault();
        last_odometry_matrix = Eigen::Affine3f::Identity();
    }

    DP convert2DP(const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
        pcl::PointCloud<PointType> pcl_scan;
        pcl::fromROSMsg(*cloudMsg, pcl_scan);

        std::vector<int> index;
        pcl::removeNaNFromPointCloud(pcl_scan, pcl_scan, index);

        std::shared_ptr<PM::DataPointsFilter> voxelFilter =
                PM::get().DataPointsFilterRegistrar.create("VoxelGridDataPointsFilter",
                                                           {{"vSizeX", toParam(0.2)},
                                                            {"vSizeY", toParam(0.2)},
                                                            {"vSizeZ", toParam(0.3)},
                                                            {"useCentroid", toParam(1)}});
        return voxelFilter->filter(convertPCL2PM(pcl_scan));
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
        TicToc t_icp;
        currentScan = convert2DP(cloudMsg);

        if(!init) {
            lastScan = currentScan;
            init = true;
        }

        static Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();

        trans_matrix = icp(currentScan, lastScan, trans_matrix);

        std::cout << "pm icp finish, " << t_icp.toc() << std::endl;

        last_odometry_matrix.matrix() = last_odometry_matrix.matrix() * trans_matrix;

        // publish pm odometry
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(last_odometry_matrix, x, y, z, roll, pitch, yaw);

        // publish latest odometry
        nav_msgs::Odometry pm_odom;
        pm_odom.header = cloudMsg->header;
        pm_odom.header.frame_id = "camera_init";
        pm_odom.pose.pose.position.x = x;
        pm_odom.pose.pose.position.y = y;
        pm_odom.pose.pose.position.z = z;
        pm_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubPmOdometry.publish(pm_odom);

        lastScan = currentScan;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "LidarPmOdometry");

    LidarPmOdometry pm_odometry;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Pm Odometry Started.");

    ros::spin();

    return 0;
}
