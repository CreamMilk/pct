#pragma once
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include "vector3.h"
#include "LeastSquare.h"

namespace pct
{
    
    typedef struct lineinfo__
    {
        lineinfo__():begin_tower_no(-1), end_tower_no(-1){};
        enum Asix{ X = 0, Y = 1, Z = 2 };
        Vector3 center;
        Vector3 sta;
        Vector3 end;
        Vector3 v;
        Asix maxAsix;
        std::vector<Vector3> pts;
        std::vector<Vector3> fit_pts;
        pcl::PointIndices indices;
        Fit fit;
        Fit fit_z;
        int begin_tower_no;
        int end_tower_no;
        std::string getLineNo()
        {
            std::string res;
            if (begin_tower_no != -1)
            {
                std::stringstream ss;
                ss << begin_tower_no;
                res += ss.str();
            }
            res += "-";
            if (end_tower_no != -1)
            {
                std::stringstream ss;
                ss << end_tower_no;
                res += ss.str();
            }
            return res;
        }

    } LineInfo;


    typedef struct towerinfo__
    {
        towerinfo__() :tower_no(-1){};
        towerinfo__(pcl::PointIndices &in_indices) 
            :tower_no(-1)
        {
            indices = in_indices;
        }
        pcl::PointIndices indices;
        pcl::PointXYZRGB cen;
        pcl::PointXYZRGB  min;
        pcl::PointXYZRGB  max;

        int tower_no;

        std::string getNo()
        {
            std::string res;
            std::stringstream ss;
            ss << tower_no;
            res += ss.str();
           
            res += "#";
            return res;
        }
    } TowerInfo;
  
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

