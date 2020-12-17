#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include "lidarFactor.hpp"
#include <glog/logging.h>
#include "hdmap_poly.h"
#include "pm_utility.h"

typedef pcl::PointXYZI  PointType;
#define DISTORTION 0

struct MatcherFrame {
    typedef std::shared_ptr<MatcherFrame> Ptr;
    typedef std::shared_ptr<const MatcherFrame> ConstPtr;

    double timestamp;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;
    pcl::PointCloud<PointType> fullPoints;

    std::string Str() const {
      return "- timestamp: " + std::to_string(timestamp) +
             ", cornerPointsSharp_ size: " + std::to_string(cornerPointsSharp.size()) +
             ", less sharp size: " + std::to_string(cornerPointsLessSharp.size()) +
             ", flat size: " + std::to_string(surfPointsFlat.size()) +
             ", less flat size: " + std::to_string(surfPointsLessFlat.size());
    }
};

struct MotionParams {
    double t_x;
    double t_y;
    double t_z;

    double q_x;
    double q_y;
    double q_z;
    double q_w;

    std::string Str() {
      return "tx: " + std::to_string(t_x) +
              ", ty: " + std::to_string(t_y) +
              ", tz: " + std::to_string(t_z) +
              ", qx: " + std::to_string(q_x) +
              ", qy: " + std::to_string(q_y) +
              ", qz: " + std::to_string(q_z) +
              ", qw: " + std::to_string(q_w);
    }
};


class Scan2ScanOdomEstimation {
public:
    struct CornerConrrespondence {
        double s;
        Eigen::Vector3d s_p;
        Eigen::Vector3d t_p_1;
        Eigen::Vector3d t_p_2;
    };
    typedef std::vector<CornerConrrespondence> CornerConrrespondences;

    struct PlaneConrrespondence {
        double s;
        Eigen::Vector3d s_p;
        Eigen::Vector3d t_p_1;
        Eigen::Vector3d t_p_2;
        Eigen::Vector3d t_p_3;
    };
    typedef std::vector<PlaneConrrespondence> PlaneConrrespondences;

public:
    Scan2ScanOdomEstimation() {}


    void SetHDMapTaskFrame(const HDMapTaskFrame::Ptr &hdmap_polygen) {
      hdmap_polygen_ = hdmap_polygen;
    }

    void SetInitPose(const Eigen::Vector3d &init_pose) {
      this->para_t[0] = init_pose[0];
      this->para_t[1] = init_pose[1];
      this->para_t[2] = init_pose[2];
    }

    void FilterLastFrame(const Eigen::Affine3f &last_pose) {
      XYZ frame_center{last_pose(0,3), last_pose(1,3)};
      if(hdmap_polygen_->isInBufferedFrame(frame_center)) {
        pcl::PointCloud<PointType> viewPoints;
        pcl::transformPointCloud(last_frame_->fullPoints, viewPoints, last_pose.matrix());
        for(auto &p : viewPoints.points) p.intensity = 240;

        for(auto &p : viewPoints.points) {
          XYZ p_xy{p.x, p.y};

          if(hdmap_polygen_->isInBufferedFrame(p_xy)) {
            p.intensity = 10;
          }
        }

        DP view_coud = convertPCL2PM(viewPoints);
        std::string pcd_file = "/home/xl/ht_test/debug/" + std::to_string(last_frame_->timestamp) + ".pcd";
        view_coud.save(pcd_file);
        LOG(INFO) << " -- pcd_file: " << pcd_file;

        pcl::transformPointCloud(last_frame_->cornerPointsLessSharp, last_frame_->cornerPointsLessSharp, last_pose.matrix());
        auto &filter_points = last_frame_->cornerPointsLessSharp.points;
        filter_points.erase(
                remove_if(filter_points.begin(), filter_points.end(),
                          [&](const pcl::PointXYZI& pt) -> bool {
                              XYZ p_xy{pt.x, pt.y};
                              if(hdmap_polygen_->isInBufferedFrame(p_xy)) {
                                return true;
                              } else {
                                return false;
                              }
                          }), filter_points.end());
        pcl::transformPointCloud(last_frame_->cornerPointsLessSharp, last_frame_->cornerPointsLessSharp, last_pose.inverse().matrix());

        DP view_filter_points = convertPCL2PM(last_frame_->cornerPointsLessSharp);
        std::string pcd_file__ = "/home/xl/ht_test/debug/" + std::to_string(last_frame_->timestamp) + "_.pcd";
        view_filter_points.save(pcd_file__);
      }


    }


    bool Run(const MatcherFrame::Ptr &current_frame) {
      if (!initialized) {
        if(MoveCurrentFrame2LastFrame(current_frame)) {
          initialized = true;
          LOG(WARNING) << " -- Initialization Finished -- ";
          return true;

        } else
          return false;
      }

      current_frame_ = current_frame;

      if (!ScanMatch()) {
        LOG(WARNING) << "ScanMatch Failed !";
        return false;
      }

      if (!MoveCurrentFrame2LastFrame(current_frame)) {
        LOG(WARNING) << "MoveCurrentFrame2LastFrame Failed !";
        return false;
      }

      return true;
    }

    MotionParams GetResult() const {
      MotionParams result;
      result.q_x = para_q[0];
      result.q_y = para_q[1];
      result.q_z = para_q[2];
      result.q_w = para_q[3];

      result.t_x = para_t[0];
      result.t_y = para_t[1];
      result.t_z = para_t[2];
      return result;
    }

private:
    void TransformToStart(PointType const *const pi, PointType *const po) {
      //interpolation ratio
      Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
      Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

      double s;
      if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
      else
        s = 1.0;
      //s = 1;
      Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
      Eigen::Vector3d t_point_last = s * t_last_curr;
      Eigen::Vector3d point(pi->x, pi->y, pi->z);
      Eigen::Vector3d un_point = q_point_last * point + t_point_last;

      po->x = un_point.x();
      po->y = un_point.y();
      po->z = un_point.z();
      po->intensity = pi->intensity;
    }

    bool MoveCurrentFrame2LastFrame(const MatcherFrame::Ptr &current_frame) {
      last_frame_ = current_frame;

      laserCloudCornerLastNum_ = last_frame_->cornerPointsLessSharp.size();
      laserCloudSurfLastNum_ = last_frame_->surfPointsLessFlat.size();

      kdtreeCornerLast->setInputCloud(last_frame_->cornerPointsLessSharp.makeShared());
      kdtreeSurfLast->setInputCloud(last_frame_->surfPointsLessFlat.makeShared());

      return true;
    }

    bool ScanMatch() {
      for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {

        if(!ComputeConrrespondences()) {
          LOG(WARNING) << "ComputeConrrespondences Failed !";
          return false;
        }

        if(!EstimationMotion()) {
          LOG(WARNING) << "EstimationMotion Failed !";
          return false;
        }

        if(corner_conrrespondences_.size() < 15 || plane_conrrespondences_.size() < 15) {
          LOG(WARNING) << "corner_correspondence: " << corner_conrrespondences_.size()
                    << ", plane_correspondence: " << plane_conrrespondences_.size();
        }

      }
      return true;
    }

    bool ComputeCornerFeatures() {
      corner_conrrespondences_.clear();

      int cornerPointsSharpNum = current_frame_->cornerPointsSharp.size();
      const auto cornerPointsLessSharpLast = last_frame_->cornerPointsLessSharp.makeShared();
      const auto cornerPointsSharp = current_frame_->cornerPointsSharp.makeShared();


      pcl::PointXYZI pointSel;
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;

      // find correspondence for corner features
      for (int i = 0; i < cornerPointsSharpNum; ++i) {
        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;

        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
          closestPointInd = pointSearchInd[0];
          int closestPointScanID = int(cornerPointsLessSharpLast->points[closestPointInd].intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line

          for (int j = closestPointInd + 1; j < (int)cornerPointsLessSharpLast->size(); ++j) {
            // if in the same scan line, continue
            if (int(cornerPointsLessSharpLast->points[j].intensity) <= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(cornerPointsLessSharpLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (cornerPointsLessSharpLast->points[j].x - pointSel.x) *
                                (cornerPointsLessSharpLast->points[j].x - pointSel.x) +
                                (cornerPointsLessSharpLast->points[j].y - pointSel.y) *
                                (cornerPointsLessSharpLast->points[j].y - pointSel.y) +
                                (cornerPointsLessSharpLast->points[j].z - pointSel.z) *
                                (cornerPointsLessSharpLast->points[j].z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if in the same scan line, continue
            if (int(cornerPointsLessSharpLast->points[j].intensity) >= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(cornerPointsLessSharpLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (cornerPointsLessSharpLast->points[j].x - pointSel.x) *
                                (cornerPointsLessSharpLast->points[j].x - pointSel.x) +
                                (cornerPointsLessSharpLast->points[j].y - pointSel.y) *
                                (cornerPointsLessSharpLast->points[j].y - pointSel.y) +
                                (cornerPointsLessSharpLast->points[j].z - pointSel.z) *
                                (cornerPointsLessSharpLast->points[j].z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }

        // both closestPointInd and minPointInd2 is valid
        if (minPointInd2 >= 0) {
          Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                     cornerPointsSharp->points[i].y,
                                     cornerPointsSharp->points[i].z);

          Eigen::Vector3d last_point_a(cornerPointsLessSharpLast->points[closestPointInd].x,
                                       cornerPointsLessSharpLast->points[closestPointInd].y,
                                       cornerPointsLessSharpLast->points[closestPointInd].z);

          Eigen::Vector3d last_point_b(cornerPointsLessSharpLast->points[minPointInd2].x,
                                       cornerPointsLessSharpLast->points[minPointInd2].y,
                                       cornerPointsLessSharpLast->points[minPointInd2].z);

          double s;
          if (DISTORTION)
            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
          else
            s = 1.0;

          corner_conrrespondences_.emplace_back();
          corner_conrrespondences_.back().s = s;
          corner_conrrespondences_.back().s_p = curr_point;
          corner_conrrespondences_.back().t_p_1 = last_point_a;
          corner_conrrespondences_.back().t_p_2 = last_point_b;
        }
      }

      return true;
    }

    bool ComputePlaneFeatures() {
      plane_conrrespondences_.clear();
      int surfPointsFlatNum = current_frame_->surfPointsFlat.size();

      pcl::PointXYZI pointSel;
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;

      const auto laserCloudSurfLast = last_frame_->surfPointsLessFlat.makeShared();
      const auto surfPointsFlat = current_frame_->surfPointsFlat.makeShared();

      // find correspondence for plane features
      for (int i = 0; i < surfPointsFlatNum; ++i) {
        TransformToStart(&(current_frame_->surfPointsFlat.points[i]), &pointSel);
        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
          closestPointInd = pointSearchInd[0];

          // get closest point's scan ID
          int closestPointScanID = int(last_frame_->surfPointsLessFlat.points[closestPointInd].intensity);
          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)last_frame_->surfPointsLessFlat.points.size(); ++j) {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                                (laserCloudSurfLast->points[j].y - pointSel.y) *
                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                                (laserCloudSurfLast->points[j].z - pointSel.z) *
                                (laserCloudSurfLast->points[j].z - pointSel.z);

            // if in the same or lower scan line
            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
              // if in the higher scan line
            else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                (laserCloudSurfLast->points[j].x - pointSel.x) +
                                (laserCloudSurfLast->points[j].y - pointSel.y) *
                                (laserCloudSurfLast->points[j].y - pointSel.y) +
                                (laserCloudSurfLast->points[j].z - pointSel.z) *
                                (laserCloudSurfLast->points[j].z - pointSel.z);

            // if in the same or higher scan line
            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;

            } else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          if (minPointInd2 >= 0 && minPointInd3 >= 0) {
            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                       surfPointsFlat->points[i].y,
                                       surfPointsFlat->points[i].z);
            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                         laserCloudSurfLast->points[closestPointInd].y,
                                         laserCloudSurfLast->points[closestPointInd].z);
            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                         laserCloudSurfLast->points[minPointInd2].y,
                                         laserCloudSurfLast->points[minPointInd2].z);
            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                         laserCloudSurfLast->points[minPointInd3].y,
                                         laserCloudSurfLast->points[minPointInd3].z);

            double s;
            if (DISTORTION)
              s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
            else
              s = 1.0;

            plane_conrrespondences_.emplace_back();
            plane_conrrespondences_.back().s = s;
            plane_conrrespondences_.back().s_p = curr_point;
            plane_conrrespondences_.back().t_p_1 = last_point_a;
            plane_conrrespondences_.back().t_p_2 = last_point_b;
            plane_conrrespondences_.back().t_p_3 = last_point_c;
          }
        }
      }

      return true;
    }

    bool ComputeConrrespondences() {

      if (!ComputeCornerFeatures()) {
        LOG(WARNING) << "ComputeCornerFeatures Failed !";
        return false;
      }

      if (!ComputePlaneFeatures()) {
        LOG(WARNING) << "ComputePlaneFeatures Failed !";
        return false;
      }

      return true;
    }

    bool EstimationMotion() {
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization =
              new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(para_q, 4, q_parameterization);
      problem.AddParameterBlock(para_t, 3);

      for(const auto &corner : corner_conrrespondences_) {
        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(corner.s_p, corner.t_p_1, corner.t_p_2, corner.s);
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
      }

      for(const auto &plane : plane_conrrespondences_) {
        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(plane.s_p, plane.t_p_1, plane.t_p_2, plane.t_p_3, plane.s);
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      return true;
    }

private:
    bool initialized = false;

    int laserCloudCornerLastNum_ = 0;
    int laserCloudSurfLastNum_ = 0;

    MatcherFrame::Ptr current_frame_;
    MatcherFrame::Ptr last_frame_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast =
            boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast =
            boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

    CornerConrrespondences corner_conrrespondences_;
    PlaneConrrespondences plane_conrrespondences_;

    double SCAN_PERIOD = 0.1;
    double NEARBY_SCAN = 2.5;
    double DISTANCE_SQ_THRESHOLD = 25;

    int corner_correspondence = 0;
    int plane_correspondence = 0;
    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};


    HDMapTaskFrame::Ptr hdmap_polygen_ = std::make_shared<HDMapTaskFrame>();
};
