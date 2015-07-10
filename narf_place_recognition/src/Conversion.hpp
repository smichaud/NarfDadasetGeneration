#ifndef CONVERSION_H
#define CONVERSION_H

#include "Definitions.hpp"
#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

namespace Conversion {
    Eigen::Quaternionf tfToEigen(tf::Quaternion tfQuat);
    tf::Quaternion eigenToTf(Eigen::Quaternionf eigenQuat);

    tf::Vector3 eigenToTf(Eigen::Translation3f eigenTranslation);
    Eigen::Translation3f tfToEigen(tf::Vector3 tfTranslation);

    Eigen::Transform<float, 3, Eigen::Affine> tranformationFromTf(
            tf::Transform tfTransfo);
}

#endif
