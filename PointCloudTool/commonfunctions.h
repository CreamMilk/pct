#pragma once
#include <vector>
#include <string>
#include "dbscan.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <QString>
#include "types/mytypes.h"

namespace pct
{
    struct LBH
    {
        LBH()
        {
            l1 = l2 = b1 = b2 = H = l3 = b3 = 0;
        }
        int l1, l2, b1, b2;
        double  H, l3, b3;
    };
    void StringReplace(string &strBase, string strSrc, string strDes);
    double Distance3d(pcl::PointXYZRGB &pt1, pcl::PointXYZRGB &pt2);
    double Distance3d(Vector3 &pt1, Vector3 &pt2);
    double Distance2d(pcl::PointXYZRGB &pt1, pcl::PointXYZRGB &pt2);
    double Distance2d(Vector3 &pt1, pcl::PointXYZRGB &pt2);
    double Distance2d(double x, double y, double m, double n);
    std::string GetExeName();
    std::string GetExePath();
    std::string ExtractExeName(const std::string &input);
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


    bool likeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices);
    //void FindLikeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr cloud_indices, std::vector<std::vector<int>> &clusters, float dbscaneps, int dbscanmin);
    void ouShiFenGe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, const std::vector<int> &indeces, std::vector<pcl::PointIndices>& cluster_indices, double k);
    void ouShiFenGe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<pcl::PointIndices>& cluster_indices, double k);
    void mergeBalls(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCrashPoint, std::vector<pcl::PointIndices> &cluster_indices);
    //void deleteObbErrorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<int> &indeces, std::set<int> &error_points);
    void deleteObbErrorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<int> &indeces, std::vector <pcl::PointIndices> &error_points);
    void MergeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices>& jlClusters, std::vector <pct::LineInfo>& lineClusters, std::vector <pct::TowerInfo>& towerClusters);
    std::string to_utf8(const std::wstring& str);
    std::string to_utf8(const wchar_t* buffer, int len);
    std::wstring String2WString(const std::string& s);
    std::string WString2String(const std::wstring& ws);
    bool DelDir(const QString &path);
    void UTMXY2LatLon(double &x, double &y, int zone=50, bool southhemi=false);
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


