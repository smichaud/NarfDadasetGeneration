#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <numeric>

#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace fs = boost::filesystem;

void loadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);
template <typename Type> float median(std::vector<Type> values);

int main(int argc, char* argv[])
{
    std::vector<int> counts;
    counts.reserve(200);
    std::vector<float> ranges;
    ranges.reserve(340000*200);

    fs::path targetDir(argv[1]);

    fs::directory_iterator it(targetDir), eod;
    int current = 0;
    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod)) {
        if(is_regular_file(p) && it->path().extension().string() == ".pcd") {
            std::cout << "Current: " << current << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            loadCloud(cloud, it->path().string());

            counts.push_back(cloud->points.size());
            for(unsigned long int i = 0; i < cloud->size(); ++i) {
                float x = (*cloud)[i].x;
                float y = (*cloud)[i].y;
                float z = (*cloud)[i].z;
                float range = sqrt(x*x+y*y+z*z);
                ranges.push_back(range);
            }

            current++;
        }
    }

    float medianCount = median(counts);
    float sumCounts = std::accumulate(counts.begin(), counts.end(), 0.0);
    float meanCount = sumCounts/counts.size();
    std::cout << "Average count: " << meanCount << std::endl;
    std::cout << "Median count : " << medianCount << std::endl;

    float medianRange = median(ranges);
    float sumRanges = std::accumulate(ranges.begin(), ranges.end(), 0.0);
    float meanRange = sumRanges/ranges.size();
    std::cout << "Average range: " << meanRange << std::endl;
    std::cout << "Median range : " << medianRange << std::endl;
}

void loadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) != 0) {
        std::cout << "Unable to load the cloud file" << std::endl;
    }
}

template <typename Type> float median(std::vector<Type> values) {
    if(values.empty()) {
        return 0;
    } else {
        std::sort(values.begin(), values.end());
        if(values.size() % 2 == 0) {
                return (values[values.size()/2 - 1] + values[values.size()/2]) / 2;
        } else {
                return values[values.size()/2];
        }
    }
}
