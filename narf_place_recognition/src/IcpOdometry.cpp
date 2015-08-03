#include "IcpOdometry.hpp"
#include <fstream>

namespace IcpOdometry {
    //Note: Cloud origin == sensor. Therefore endCloud must be transformed :)
    Eigen::Matrix4f getCorrectedTransfo(
            const PointCloud &startCloud,
            const PointCloud &endCloud,
            Eigen::Matrix4f initTransfo,
            const std::string &configFile,
            const std::string &cloudsOutputPath,
            bool isOdomMergedCloudsSaved) {
        PM::ICP icp;

        std::ifstream configFileStream(configFile.c_str());
        if (configFile.empty() || !configFileStream.good()) {
            std::cerr << "Unable to load ICP configuration file." << std::endl;
            std::cerr << "Default setting will be used." << std::endl;
            icp.setDefault();
        } else {
            icp.loadFromYaml(configFileStream);
        }

        PM::TransformationParameters approxTransfo = initTransfo;
        PM::Transformation* rigidTrans;
        rigidTrans = PM::get().REG(Transformation).
            create("RigidTransformation");

        const PointCloud initializedCloud =
            rigidTrans->compute(endCloud, approxTransfo);
        PM::TransformationParameters errorTransfo;
        try{
            errorTransfo = icp(initializedCloud, startCloud);
        } catch(PM::ConvergenceError exception) {
            errorTransfo = Eigen::Matrix4f::Identity();
            std::cerr << "ICP was not able to converge." << std::endl;
            std::cerr << "Initial approximation of transformation will be used."
                << std::endl;
        }

        if(!cloudsOutputPath.empty()) {
            PointCloud adjustedCloud(initializedCloud);
            try {
                icp.transformations.apply(adjustedCloud, errorTransfo);
            } catch(...) {
                std::cerr << "An error occured while trying to apply ICP"
                    << std::endl;
            }

            if(isOdomMergedCloudsSaved == false) {
                startCloud.save(cloudsOutputPath + "_start_cloud.vtk");
                endCloud.save(cloudsOutputPath + "_end_cloud_ref.vtk");
                initializedCloud.save(cloudsOutputPath + "_init_cloud.vtk");
                adjustedCloud.save(cloudsOutputPath + "_final_cloud.vtk");
            } else {
                PointCloud startCloudCopy(startCloud);
                saveMergedClouds(cloudsOutputPath, startCloudCopy,
                        adjustedCloud);
            }
        }

        return errorTransfo*approxTransfo;
    }

    void saveMergedClouds(const std::string &cloudsOutputPath,
            PointCloud &startCloud, PointCloud &endCloud) {
        boundingBox(startCloud);
        boundingBox(endCloud);

        if(!startCloud.descriptorExists("intensity")) {
            uli pointsCount = startCloud.getNbPoints();
            startCloud.addDescriptor("intensity",
                    PM::Matrix::Zero(1,pointsCount));
        }
        if(!endCloud.descriptorExists("intensity")) {
            uli pointsCount = endCloud.getNbPoints();
            endCloud.addDescriptor("intensity",
                    PM::Matrix::Ones(1,pointsCount));
        }

        unsigned int startDescIndex =
            startCloud.getDescriptorStartingRow("intensity");
        unsigned int endDescIndex =
            endCloud.getDescriptorStartingRow("intensity");
        uli startPointsCount = startCloud.getNbPoints();
        uli endPointsCount = endCloud.getNbPoints();

        startCloud.descriptors.block(startDescIndex, 0, 1, startPointsCount) =
            PM::Matrix::Zero(1, startPointsCount);
        endCloud.descriptors.block(endDescIndex, 0, 1, endPointsCount) =
            PM::Matrix::Ones(1, endPointsCount);

        startCloud.concatenate(endCloud);
        startCloud.save(cloudsOutputPath + ".vtk");
    }

    void boundingBox(PointCloud &pointCloud,
            float xMin, float xMax,
            float yMin, float yMax,
            float zMin, float zMax,
            bool removeInside) {
        PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "BoundingBoxDataPointsFilter",
                    PointMatcherSupport::map_list_of
                    ("xMin", PointMatcherSupport::toParam(xMin))
                    ("xMax", PointMatcherSupport::toParam(xMax))
                    ("yMin", PointMatcherSupport::toParam(yMin))
                    ("yMax", PointMatcherSupport::toParam(yMax))
                    ("zMin", PointMatcherSupport::toParam(zMin))
                    ("zMax", PointMatcherSupport::toParam(zMax))
                    ("removeInside",
                     PointMatcherSupport::toParam(removeInside))));

        filter->inPlaceFilter(pointCloud);
    }
}
