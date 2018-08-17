#pragma once
#include <QFileInfo>
#include "Scene_points_with_normal_item.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pct
{
    namespace io
    {
        // º”‘ÿlas
        Scene_points_with_normal_item* lasload(const std::string& file_path);

        // ±£¥Êlas
        bool lassave(const Scene_points_with_normal_item* item, const std::string& file_path);


        void save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
        void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);

    }
   
}

