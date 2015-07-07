#ifndef ICPODOMETRY_H
#define ICPODOMETRY_H

#include "Definitions.hpp"

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace icpodometry {
    Transformation getTransfo(
            const PointCloud &startCloud,
            const PointCloud &endCloud,
            Transformation initTransfo,
            const std::string &configFile);
}

#endif
