#ifndef DEFINITION_H
#define DEFINITION_H

#include <pointmatcher/PointMatcher.h>
#include <Eigen/Dense>

typedef PointMatcher<float> PM;
typedef PM::DataPoints PointCloud;
typedef Eigen::Matrix4f Transformation;

#endif
