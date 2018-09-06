#pragma once
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/PointIndices.h>
 #include <boost/make_shared.hpp>
 #include <vector>
 #include <string>
 #include "QRendView.h"

 typedef struct
 {
     pcl::PointXYZRGB cen;
     double radiu;
     std::string id;
     std::string description;
 }CollisionBall;

 class DangerousDistanceCheck
 {
 public:
     DangerousDistanceCheck();
     ~DangerousDistanceCheck();
     void TooNearCheck();
     void showNearCheck();
     void setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices> &otherClusters, std::vector <pcl::PointIndices> &lineClusters, std::vector <pcl::PointIndices> &towerClusters, double dangerousDistance);
     void LabelCrashLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint, double K, double gap);
     std::vector<CollisionBall> balls_;//Åö×²ÇòÐÎ±ê×¢
     
 
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_;
     std::vector <pcl::PointIndices> otherClusters_;
     std::vector <pcl::PointIndices> lineClusters_;
     std::vector <pcl::PointIndices> towerClusters_;
     pcl::PointIndicesPtr ground_indices_;
     double dangerousDistance_;
 };
