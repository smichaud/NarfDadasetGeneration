#ifndef CONVERSION_H
#define CONVERSION_H

#include "Definitions.hpp"
#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

namespace Conversion {
    Transformation tranformationFromTf(tf::Transform tfTransfo);
}

#endif
