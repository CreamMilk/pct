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
        enum Asix{ X = 0, Y = 1, Z = 2 };
        Vector3 center;
        Vector3 sta;
        Vector3 end;
        Vector3 v;
        Asix maxAsix;
        std::vector<Vector3> pts;
        std::vector<Vector3> fit_pts;
        Fit fit;
        Fit fit_z;
    } LineInfo;
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

