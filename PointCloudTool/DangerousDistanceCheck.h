#pragma once
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/PointIndices.h>
 #include <boost/make_shared.hpp>
 #include <vector>
 #include <string>
#include "MathGeoLibFwd.h"
#include "Math/float3.h"

 class DangerousDistanceCheck
 {
 public:
     DangerousDistanceCheck();
     ~DangerousDistanceCheck();

     enum CrashType  {NoCrash = 0, Ground = 1, Other = 2 };

     typedef struct ___CollisionBall
     {
         ___CollisionBall(){ crashtype = 0; };
         pcl::PointXYZRGB cen;  // Åö×²µã
         pcl::PointXYZRGB min;
         pcl::PointXYZRGB max;
         double radiu;          // ¼ì²â·¶Î§
         unsigned int crashtype;
         double overtoplimit;

         struct ___Nearst
         {
             pcl::PointXYZRGB linept;
             pcl::PointXYZRGB otherpt;
             vec subVec;
             double dis;
         } nearst;

         std::string id;
         std::string description;
     }CollisionBall;

     void TooNearCheck();
     void showNearCheck();
     void setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices> &otherClusters, std::vector <pcl::PointIndices> &lineClusters, std::vector <pcl::PointIndices> &towerClusters, double dangerousDistance);
     std::vector<CollisionBall> balls_;//Åö×²ÇòÐÎ±ê×¢
     
     
 
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_;
     std::vector <pcl::PointIndices> otherClusters_;
     std::vector <pcl::PointIndices> lineClusters_;
     std::vector <pcl::PointIndices> towerClusters_;
     pcl::PointIndicesPtr ground_indices_;
     double dangerousDistance_;
 };
