#ifndef ICPODOMETRY_H
#define ICPODOMETRY_H

#include "Definitions.hpp"

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace IcpOdometry {
    Eigen::Matrix4f getCorrectedTransfo(
            const PointCloud &startCloud,
            const PointCloud &endCloud,
            const Eigen::Matrix4f initTransfo,
            const std::string &configFile,
            const std::string &cloudsOutputPath = "",
            const bool isOdomMergedCloudsSaved = false);

    void saveMergedClouds(const std::string &cloudsOutputPath,
            PointCloud &startCloud, PointCloud &endCloud);

    void boundingBox(PointCloud &pointCloud,
            float xMin = -20, float xMax = 20,
            float yMin = -20, float yMax = 20,
            float zMin = 0, float zMax = 4.0,
            bool removeInside = 0);
}

#endif
