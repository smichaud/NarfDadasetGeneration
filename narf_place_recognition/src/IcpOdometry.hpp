#ifndef ICPODOMETRY_H
#define ICPODOMETRY_H

#include <iostream>
#include "pointmatcher/PointMatcher.h"
#include "boost/shared_ptr.hpp"
#include <Eigen/Dense>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

namespace icpodometry {
    void getOdom(DP cloud1, DP cloud2, std::string file);
};

#endif
