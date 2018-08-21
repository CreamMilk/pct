#pragma once
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include "types/lineInfo.h"
#include <QString>


namespace pct
{
    bool combineTrainXmlFiles(std::vector<std::string> xmls, std::string dst_xml);
    void simple(std::string inputfile, std::string outputfile, float gridsize);

    void OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices = nullptr);
    void colorClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::PointIndices>& jlClusters);
    void colorClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointIndices& inputindices, std::vector <pcl::PointIndices>& jlClusters);
    unsigned int pointsCountsForColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& clusters, const unsigned int color);
    pct::LineInfo lineInfoFactory(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices);
    bool LikePowerLine(pct::LineInfo &line, int min_length = 5, double error_probability = 0.1, float yerrOffset = 1.0f, float zerrOffset = 0.5f);
    unsigned int colorstr2int(QString c);
    bool LikePowerLine1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud, pct::LineInfo &line, int min_length /*= 5*/, double error_probability /*= 0.1*/, float yerrOffset /*= 1.0f*/, float zerrOffset /*= 0.5f*/);
    void getMinMax3D(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const pcl::PointIndices &indices,
        pcl::PointXYZRGB &min_pt, pcl::PointXYZRGB &max_pt);
    void ExtractGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices, pcl::PointIndicesPtr ground_indices);

    bool likeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<uint> indices);
}












#define setsettingitem(itemname, itemtpye, must, def) \
    itemtpye itemname;\
if (vm.count(#itemname)){ \
    std::cout << "[" << #itemname << "]\t" << vm[#itemname].as<itemtpye>() << std::endl;  \
    itemname = vm[#itemname].as<itemtpye>();                        \
}else {         \
    if (must) {\
          std::cout << "输入的参数中未找到" << #itemname << "选项！" << std::endl;           \
          return false;\
                                                            }else{\
            itemname = def;\
                                                            }\
 }\
setting.itemname = itemname;


