#include "Conversion.hpp"

namespace Conversion {
    Transformation tranformationFromTf(
            tf::Transform tfTransfo) {
        tf::Quaternion tfQuat = tfTransfo.getRotation();
        Eigen::Quaternionf eigenQuat = Eigen::Quaternionf(
                tfQuat.w(), tfQuat.x(), tfQuat.y(), tfQuat.z());

        tf::Vector3 tfTranslation = tfTransfo.getOrigin();
        Eigen::Vector3f translation = Eigen::Vector3f(
                tfTranslation.x(), tfTranslation.y(), tfTranslation.z());

        return Eigen::Matrix4f::Identity();
    }
}
