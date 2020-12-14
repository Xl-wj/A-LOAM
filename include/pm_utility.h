//
// Created by xl on 20-12-11.
//

#ifndef ALOAM_VELODYNE_PM_UTILITY_H
#define ALOAM_VELODYNE_PM_UTILITY_H

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Parametrizable.h>
#include "lego_loam_utility.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
using namespace PointMatcherSupport;


DP convertPCL2PM(const pcl::PointCloud<PointType> &cloud) {
  std::size_t point_number = cloud.size();

  DP pm_points;

  pm_points.features.resize(4, point_number);
  pm_points.featureLabels.push_back(DP::Label("x", 1));
  pm_points.featureLabels.push_back(DP::Label("y", 1));
  pm_points.featureLabels.push_back(DP::Label("z", 1));
  pm_points.featureLabels.push_back(DP::Label("pad", 1));

  pm_points.descriptors.resize(1, point_number);
  pm_points.descriptorLabels.push_back(DP::Label("intensity", 1));

  for (std::size_t i = 0; i < point_number; i++) {
    pm_points.features(0, i) = cloud.points[i].x;
    pm_points.features(1, i) = cloud.points[i].y;
    pm_points.features(2, i) = cloud.points[i].z;
    pm_points.features(3, i) = 1.0;

    pm_points.descriptors(0, i) = cloud.points[i].intensity;
  }

  return pm_points;
}

#endif //ALOAM_VELODYNE_PM_UTILITY_H
