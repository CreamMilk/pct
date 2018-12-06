#pragma once
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/PointIndices.h>
 #include <boost/make_shared.hpp>
 #include <vector>
 #include <string>
#include "MathGeoLibFwd.h"
#include "Math/float3.h"
#include <iostream>
#include "types/mytypes.h"

 class DangerousDistanceCheck
 {
 public:
     DangerousDistanceCheck();
     ~DangerousDistanceCheck();

     enum CrashType  {NoCrash = 0, Ground = 1, Other = 2 };

     class CollisionBall
     {
     public:
		 CollisionBall(){ crashtype = 0; ground_distance = 0; overtoplimit = 0; nearst_limit_type = 0; };
         double GetExtraBoxRadiu(const pcl::PointXYZRGB &pt);
         pcl::PointXYZRGB cen;  // 碰撞点
         pcl::PointXYZRGB min;
         pcl::PointXYZRGB max;
         double radiu;          // 检测范围
         int crashtype;
         double overtoplimit;
         double ground_distance;
         struct ___Nearst
         {
             pcl::PointXYZRGB linept;
             pcl::PointXYZRGB otherpt;
             vec subVec;
             double dis;
         } nearst;

         std::string lineno;
         std::string id;
         std::string description;
		 int nearst_limit_type;	// 0，地面  1，植被
		 std::string nearst_limit_dir_type;	// 最近点的危险方向

         friend std::ostream& operator << (std::ostream&, CollisionBall&);  //运算符“<<”重载为友元函数
     };

     void TooNearCheck();
     void showNearCheck();
     void setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pct::VegetInfo> &vegetClusters, std::vector <pct::LineInfo> &lineClusters, std::vector <pct::TowerInfo> &towerClusters);
     void getObbInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, OBB &obb, vec *axis_vec/*[3]*/, vec &axis_r, double midz);
     std::vector<CollisionBall> balls_;//碰撞球形标注
     
     
 
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud_;
     std::vector <pct::VegetInfo> vegetClusters_;
     std::vector <pct::LineInfo> lineClusters_;
     std::vector <pct::TowerInfo> towerClusters_;
     pcl::PointIndicesPtr ground_indices_;
 };

 std::ostream& operator << (std::ostream& output, DangerousDistanceCheck::CollisionBall& c);
