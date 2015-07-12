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
            Eigen::Matrix4f initTransfo,
            const std::string &configFile,
            const std::string &cloudsOutputPath = "");
}

#endif
