#include "commonfunctions.h"
#include <boost/foreach.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <map>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/uniform_sampling.h>   //均匀采样
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/src/Core/Matrix.h>
#include <pcl/filters/voxel_grid.h>
#include "pctio.h"
#include <Geometry/OBB.h>
#include <pcl/segmentation/extract_clusters.h>
#include <windows.h>
#include <string.h>

#include <QDir>
#include <QFileInfo>

struct ClusterInfo{
    Vector3 center;
    Vector3 min;
    Vector3 max;
};

/*将str1字符串中第一次出现的str2字符串替换成str3*/
void replaceFirst(char *str1, char *str2, char *str3)
{
    char *str4 = new char[strlen(str1) + 1];
    char *p;
    strcpy(str4, str1);
    if ((p = strstr(str1, str2)) != NULL)/*p指向str2在str1中第一次出现的位置*/
    {
        while (str1 != p&&str1 != NULL)/*将str1指针移动到p的位置*/
        {
            str1++;
        }
        str1[0] = '/0';/*将str1指针指向的值变成/0,以此来截断str1,舍弃str2及以后的内容，只保留str2以前的内容*/
        strcat(str1, str3);/*在str1后拼接上str3,组成新str1*/
        strcat(str1, strstr(str4, str2) + strlen(str2));/*strstr(str4,str2)是指向str2及以后的内容(包括str2),strstr(str4,str2)+strlen(str2)就是将指针向前移动strlen(str2)位，跳过str2*/
    }
    delete str4;
}
/*将str1出现的所有的str2都替换为str3*/
void replace(char *str1, char *str2, char *str3)
{
    while (strstr(str1, str2) != NULL)
    {
        replaceFirst(str1, str2, str3);
    }
}

void pct::StringReplace(string &strBase, string strSrc, string strDes)
{
    string::size_type pos = 0;
    string::size_type srcLen = strSrc.size();
    string::size_type desLen = strDes.size();
    pos = strBase.find(strSrc, pos);
    while ((pos != string::npos))
    {
        strBase.replace(pos, srcLen, strDes);
        pos = strBase.find(strSrc, (pos + desLen));
    }
}

std::string pct::GetExePath()
{
    std::string exe_path = "";
    //获取应用程序目录
    char szapipath[MAX_PATH] = { 0 };//（D:\Documents\Downloads\TEST.exe）
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //replace(szapipath, "\\", "/");
    exe_path = szapipath;
    return exe_path;
}

std::string pct::GetExeName()
{
    std::string exe_path = "";
    //获取应用程序目录
    char szapipath[MAX_PATH] = { 0 };//（D:\Documents\Downloads\TEST.exe）
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //获取应用程序名称
    char szExe[MAX_PATH] = { 0 };//（TEST.exe）
    char *pbuf = nullptr;
    char* szLine = strtok_s(szapipath, "\\", &pbuf);
    while (NULL != szLine)
    {
        strcpy_s(szExe, szLine);
        szLine = strtok_s(NULL, "\\", &pbuf);
    }
    exe_path = szExe;
    return exe_path;
}

std::string pct::ExtractExeName(const std::string &input)
{
    std::string exe_path = "";
    //获取应用程序目录
    char szapipath[MAX_PATH] = { 0 };//（D:\Documents\Downloads\TEST.exe）
    strcpy_s(szapipath, input.c_str());
    //获取应用程序名称
    char szExe[MAX_PATH] = { 0 };//（TEST.exe）
    char *pbuf = nullptr;
    char* szLine = strtok_s(szapipath, "\\", &pbuf);
    while (NULL != szLine)
    {
        strcpy_s(szExe, szLine);
        szLine = strtok_s(NULL, "\\", &pbuf);
    }
    exe_path = szExe;
    exe_path = exe_path.substr(0, exe_path.find('.'));
    return exe_path;
}

bool pct::combineTrainXmlFiles(std::vector<std::string> xmls, std::string dst_xml)
{
    bool one_label = false;
    if (boost::filesystem::path(xmls[0]).parent_path() == boost::filesystem::path(dst_xml).parent_path())
    {
        one_label = true;
    }

    struct trainXml
    {
        std::map<std::string, double> features;
        std::map<std::string, std::map<std::string, std::string>>labels;
    };

    size_t xmlsize = xmls.size();  // 几个训练文件
    std::map<std::string, std::map<std::string, std::vector<int>>> effects;  // label 特征 效果等级计数
    std::string labelname;  // 多重for循环太多了， labelname重用一下吧，用的时候注意不要冲突
    int fit_index=-1;
    int fit_num = -1;

    // 装载xml文件到结构体
    trainXml combinetrain;
    std::vector<trainXml> train_xml_vec;
    std::map<std::string, int> features_count;
    
    std::cout << dst_xml << std::endl;
    BOOST_FOREACH(std::string xml_path, xmls)
    {
        std::cout << xml_path << std::endl;
        trainXml train_xml;
        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(xml_path, pt);
        BOOST_AUTO(features, pt.get_child("classification.features"));
        BOOST_AUTO(labels, pt.get_child("classification.labels"));

        for (BOOST_AUTO(feature, features.begin()); feature != features.end(); ++feature)
        {
            train_xml.features[feature->second.get<std::string>("name")] = feature->second.get<double>("weight");
        }
        for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
        {
            labelname = label->second.get<std::string>("name");

            std::map<std::string, std::string> freture_map;
            for (BOOST_AUTO(pos, label->second.begin()); pos != label->second.end(); ++pos)  //boost中的auto
            {
                if (pos->first == "feature")
                {
                    std::cout << "name：" << pos->second.get<std::string>("name") << std::endl;
                    std::cout << "effect：" << pos->second.get<std::string>("effect") << std::endl;
                    freture_map[pos->second.get<std::string>("name")] = pos->second.get<std::string>("effect");
                }
            }
            train_xml.labels[labelname] = freture_map;
        }
        train_xml_vec.push_back(train_xml);
    }


    // 整理合并结构体
    for (auto train_xml : train_xml_vec)
    {
        for (auto it = train_xml.features.begin(); it != train_xml.features.end(); ++it)
        {
            combinetrain.features[it->first] += it->second;
            ++features_count[it->first];
        }
        for (auto it = train_xml.labels.begin(); it != train_xml.labels.end(); ++it)
        {
            labelname = it->first;
            for (auto itt = it->second.begin(); itt != it->second.end(); ++itt)
            {
                std::string effect_name = itt->first;
                auto &effect_vec = effects[labelname][effect_name];
                if (3 != effect_vec.size())
                    effect_vec.resize(3);
                if ("penalized" == itt->second)
                    effect_vec[0] ++;
                if ("neutral" == itt->second)
                    effect_vec[1] ++;
                if ("favored" == itt->second)
                    effect_vec[2] ++;
            }
        }
    }

    for (auto it = combinetrain.features.begin(); it != combinetrain.features.end(); ++it)
    {
        it->second /= features_count[it->first];
    }

    // 效果 
    for (auto it = effects.begin(); it != effects.end(); ++it)
    {
        labelname = it->first;  // 标签名
        std::map<std::string, std::string> &effect = combinetrain.labels[labelname];  // 效果结果
        for (auto itt = it->second.begin(); itt != it->second.end(); ++itt)
        {
            auto &effect_name = itt->first;  // 效果名
            auto &effect_vec = itt->second;  // 效果计数
            fit_num = -1;
            fit_index = -1;
            for (int i = 0; i < 3; ++i)
            {
                if (effect_vec[i] > fit_num)
                {
                    fit_num = effect_vec[i];
                    fit_index = i;
                }
            }
            if (0 == fit_index)
            {
                if (one_label && fit_num < (xmlsize)*0.5)
                    effect[effect_name] = "neutral";
                else
                    effect[effect_name] = "penalized";
            }
            else if(2 == fit_index)
            {
                if (one_label && fit_num < (xmlsize)*0.5)
                    effect[effect_name] = "neutral";
                else
                    effect[effect_name] = "favored";
            }
            else
            {
                effect[effect_name] = "neutral";
            }
        }
    }
    auto unselectit = combinetrain.labels.find("unselect");
    if (combinetrain.labels.size() > 2 && unselectit != combinetrain.labels.end())
        combinetrain.labels.erase(unselectit);

    boost::property_tree::ptree tree;
    for (auto it = combinetrain.features.begin(); it != combinetrain.features.end(); ++it)
    {
        boost::property_tree::ptree ptr;

        ptr.put("name", it->first);
        ptr.put("weight", it->second);
        tree.add_child("classification.features.feature", ptr);
    }

    for (auto it = combinetrain.labels.begin(); it!= combinetrain.labels.end(); ++it)
    {
        boost::property_tree::ptree ptr;
        ptr.put("name", it->first);
        auto &effs = it->second;
        for (auto itt = effs.begin(); itt != effs.end(); ++itt)
        {
            boost::property_tree::ptree ptr2;
            ptr2.put("name", itt->first);
            ptr2.put("effect", itt->second);
            ptr.add_child("feature", ptr2);
        }
        tree.add_child("classification.labels.label", ptr);
    }

    std::ofstream f(dst_xml);
    boost::property_tree::write_xml(f, tree,
#if BOOST_VERSION >= 105600
        boost::property_tree::xml_writer_make_settings<std::string>(' ', 3));
#else
        boost::property_tree::xml_writer_make_settings<char>(' ', 3));
#endif
    return true;
}

void pct::simple(std::string inputfile, std::string outputfile, float gridsize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pct::io::Load_las(cloud, inputfile);

    //均匀采样点云并提取关键点      体素下采样，重心代替
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);  //输入点云
    uniform_sampling.setRadiusSearch(gridsize);   //设置半径 
    uniform_sampling.filter(*cloud);   //滤波


    //pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    //grid.setLeafSize(gridsize, gridsize, gridsize);
    //grid.setInputCloud(cloud);
    //grid.filter(*cloud);

    pct::io::save_las(cloud, outputfile);
}

//void pct::OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    // 离群点
//    int liqunK = 10;
//    double avgDistance = 10;
//    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
//    //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
//    sor.setInputCloud(cloud);                           //设置待滤波的点云
//    sor.setMeanK(liqunK);                               //设置在进行统计时考虑查询点临近点数
//    sor.setStddevMulThresh(avgDistance);                      //设置判断是否为离群点的阀值
//    sor.filter(*cloud);                    //存储
//}

void pct::OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices /*= nullptr*/)
{
    // 离群点
    int liqunK = 10;
    double avgDistance = 10;
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
    sor.setInputCloud(cloud);                           //设置待滤波的点云
    if (cloud_indices)
         sor.setIndices(cloud_indices);
    sor.setMeanK(liqunK);                               //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(avgDistance);                      //设置判断是否为离群点的阀值

    if (cloud_indices)
        sor.filter(cloud_indices->indices);                    //存储
    else
        sor.filter(*cloud);
}

// 将点云存入二维数组
void PutPointCloud2Arr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr ground_indices, std::vector<std::vector<pcl::PointIndices::Ptr>> &pointsArr,
    int row, int col, pcl::PointXYZRGB min, pcl::PointXYZRGB max, double gridSize)
{
    // 填充网格
    for (int i = 0; i < ground_indices->indices.size(); ++i)
    {
        pcl::PointXYZRGB &pt = cloud->at(ground_indices->indices[i]);
        pcl::PointIndices::Ptr &indices = pointsArr[(int)((pt.x - min.x) / gridSize)][(int)((pt.y - min.y) / gridSize)];
        if (!indices)
            indices = boost::make_shared<pcl::PointIndices>();
        indices->indices.push_back(ground_indices->indices[i]);
    }
}

int Otsu(std::vector<double> &hist)
{
    if (!hist.size())
        return 0;


    int Histogramdim = hist.size();

    double *omega = new double[Histogramdim]{ 0 };
    double *mu = new double[Histogramdim]{ 0 };

    omega[0] = hist[0];
    mu[0] = 0;
    for (int i = 1; i < Histogramdim; i++)
    {
        omega[i] = omega[i - 1] + hist[i]; //累积分布函数
        mu[i] = mu[i - 1] + i * hist[i];
    }
    double mean = mu[Histogramdim - 1];// 灰度平均值
    double max = 0;
    int k_max = 0;
    for (int k = 1; k < Histogramdim - 1; k++)
    {
        double PA = omega[k]; // A类所占的比例
        double PB = 1 - omega[k]; //B类所占的比例
        double value = 0;
        if (fabs(PA) > 0.001 && fabs(PB) > 0.001)
        {
            double MA = mu[k] / PA; //A 类的灰度均值
            double MB = (mean - mu[k]) / PB;//B类灰度均值
            value = PA * (MA - mean) * (MA - mean) + PB * (MB - mean) * (MB - mean);//类间方差

            if (value > max)
            {
                max = value;
                k_max = k;
            }
        }
    }
    return k_max;
}

void pct::ExtractGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices, pcl::PointIndicesPtr ground_indices)
{
    // 地面提取
    int windowsize = 2;
    float slope = 1;
    float minlDistance = 5;
    float maxlDistance = 30;

    // RecoverGround
    double gridSize = 20;
    double heightSize = 1;


    /********** 粗提取地面start **************/
    //pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
    //pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);

    //	生成形态滤波器
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(windowsize);
    pmf.setSlope(slope);
    pmf.setInitialDistance(minlDistance);
    pmf.setMaxDistance(maxlDistance);

    std::cout << "粗提取地面" << std::endl;
    //提取地面
    pmf.extract(ground_indices->indices);
    cloud_indices->indices.reserve(cloud->size() - ground_indices->indices.size());
    std::set<int >ground_indices_set(ground_indices->indices.begin(), ground_indices->indices.end());
    for (int i = 0; i < cloud->size(); ++i)
    {
        if (ground_indices_set.find(i) == ground_indices_set.end())
            cloud_indices->indices.push_back(i);
    }

    
    // 从标号到点云
    //pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    //extract.setInputCloud(cloud);
    //extract.setIndices(groundindices);
    //extract.setNegative(true);
    //extract.filter(*cloud);
    //extract.setNegative(false);
    //extract.filter(*ground);
    /********** 粗提取地面end **************/




    /********** 离群点过滤start **************/
    //std::cout << "离群点过滤" << std::endl;
    //OutlierRemoval(cloud, cloud_indices);
    /********** 离群点过滤end **************/



    /********** 排除错误的地面点start **************/
    if (!ground_indices->indices.size())
    {
        std::cout<< ("地面为空？？？");
    }
    else
    {
        std::cout << "误判恢复" << std::endl;
        pcl::PointIndices::Ptr ground_erase(new pcl::PointIndices);
        std::vector<pcl::PointIndices::Ptr> towerIndies;

        // 计算二维数组的行列值
        pcl::PointXYZRGB min, max;
        pct::getMinMax3D(*cloud, *ground_indices, min, max);
        int row = (max.x - min.x) / gridSize + 1;
        int col = (max.y - min.y) / gridSize + 1;

        std::cout << "点云数量" << cloud->size() << "\t地面点索引数量 " << ground_indices->indices.size()<< std::endl;
        std::cout << "初始化平面网格" << (int)max.x << " " << (int)max.y << std::endl;
        // 初始化平面网格
        std::vector<std::vector<pcl::PointIndices::Ptr>>pointsArr(row, std::vector<pcl::PointIndices::Ptr>(col));
        std::cout << ("申请网格数组") << std::endl;
        PutPointCloud2Arr(cloud, ground_indices, pointsArr, row, col, min, max, gridSize);

        std::cout << "遍历平面网格" << std::endl;
        // 遍历每一个网格
#pragma omp parallel for
        for (int i = 0; i < pointsArr.size(); ++i)
        {
            std::vector<int> tempIndices;
            for (int j = 0; j < pointsArr[i].size(); ++j)
            {
                pcl::PointIndices::Ptr planeIndices = pointsArr[i][j];
                if (!planeIndices)
                {
                    continue;
                }

                // 准备网格数据
                pcl::PointCloud<pcl::PointXYZRGB> planeCloud;
                pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
                extract1.setInputCloud(cloud);
                extract1.setIndices(planeIndices);
                extract1.filter(planeCloud);

                // 计算网格信息
                pcl::PointXYZRGB min, max;
                pcl::getMinMax3D(planeCloud, min, max);
                int heightLevel = (max.z - min.z) / heightSize + 1;
                int planeCloudSize = planeIndices->indices.size();
                double sumzCloud = 0;

                // 统计高度直方图占比
                std::set<int> heighthistset;
                for (int k = 0; k < planeCloudSize; ++k)
                {
                    int index = (cloud->at(planeIndices->indices[k]).z - min.z) / heightSize;
                    if (heighthistset.find(index) == heighthistset.end())
                    {
                        sumzCloud += index;
                        heighthistset.insert(index);
                    }
                }

                // 高度网格有点的占比小于0.7，则认为有天空点，进行高度自动阈值过滤
                int histSize = heighthistset.size();
                if (histSize < heightLevel * 0.7)
                {
                    std::vector<double> hist;
                    for each (int var in heighthistset)
                    {
                        hist.push_back((var) / sumzCloud);
                    }

                    double threshold = Otsu(hist) * heightSize + min.z;
                    //std::cout << "地面误判：网格[%d][%d]的最高点=%f，最低点=%f，阈值%f", i, j, max.z, min.z, threshold;

                    for (std::vector<int>::iterator it = planeIndices->indices.begin(); it != planeIndices->indices.end(); ++it)
                    {
                        if (cloud->at(*it).z > threshold)
                        {
                            tempIndices.push_back(*it);
                        }
                    }
                }
            }
#pragma omp critical 
            ground_erase->indices.insert(ground_erase->indices.end(), tempIndices.begin(), tempIndices.end());
        }

        std::cout << "遍历平面网格结束" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract1;

        ground_indices_set.clear();
        std::set<int>::iterator ground_indices_eraseit;
        ground_indices_set.insert(ground_indices->indices.begin(), ground_indices->indices.end());
        for (int i = 0; i < ground_erase->indices.size(); ++i)
        {
            cloud_indices->indices.push_back(ground_erase->indices[i]);
            ground_indices_eraseit = ground_indices_set.find(ground_erase->indices[i]);
            if (ground_indices_eraseit != ground_indices_set.end())
                ground_indices_set.erase(ground_indices_eraseit);
        }
        ground_indices->indices.clear();
        ground_indices->indices.insert(ground_indices->indices.end(), ground_indices_set.begin(), ground_indices_set.end());
    }
    /********** 排除错误的地面点end **************/

    // 地物提取  貌似这不需要
    // RoughExtractGroundObject();

    //pcl::PointXYZRGB *pt;
    //for (int i = 0; i < ground_indices->indices.size(); ++i)
    //{
    //    pt = &cloud->at(ground_indices->indices[i]);
    //    pt->r = 0;
    //    pt->g = 64;
    //    pt->b = 128;
    //}
    //
    //for (int i = 0; i <cloud_indices->indices.size(); ++i)
    //{
    //    pt = &cloud->at(cloud_indices->indices[i]);
    //    pt->r = 255;
    //    pt->g = 0;
    //    pt->b = 0;
    //}
}

void pct::colorClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::PointIndices>& jlClusters)
{
    float distanceThreshold = 0.5;
    float pointColorThreshold = 0;
    float regionColorThreshold = 0;
    int minClusterSize = 50;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;//创建基于颜色的区域生长分割类的对象
    reg.setInputCloud(cloud);//设置分割原始点云
    reg.setSearchMethod(tree);//设置搜索方法，最近临搜索
    reg.setDistanceThreshold(/*10*/distanceThreshold);//设置距离阈值，小于该值的视为邻域点
    reg.setPointColorThreshold(/*8*/pointColorThreshold);//设置点之间的色差阈值，小于该值的视为一个聚类
    reg.setRegionColorThreshold(/*15*/regionColorThreshold);//设置聚类之间的色差阈值，小于该值的应用合并算法，合并为同一个聚类
    reg.setMinClusterSize(/*200*/minClusterSize);//设置聚类中点的数量下限，如果点数量少于该值，应用合并算法，合并到最近临的一个聚类

    reg.extract(jlClusters);
}

void pct::colorClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointIndices& inputindices, std::vector <pcl::PointIndices>& jlClusters)
{
    float distanceThreshold = 0.5;
    float pointColorThreshold = 0;
    float regionColorThreshold = 0;
    int minClusterSize = 50;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;//创建基于颜色的区域生长分割类的对象
    reg.setInputCloud(cloud);//设置分割原始点云
    reg.setIndices(boost::make_shared<pcl::PointIndices>(inputindices));
    reg.setSearchMethod(tree);//设置搜索方法，最近临搜索
    reg.setDistanceThreshold(/*10*/distanceThreshold);//设置距离阈值，小于该值的视为邻域点
    reg.setPointColorThreshold(/*8*/pointColorThreshold);//设置点之间的色差阈值，小于该值的视为一个聚类
    reg.setRegionColorThreshold(/*15*/regionColorThreshold);//设置聚类之间的色差阈值，小于该值的应用合并算法，合并为同一个聚类
    reg.setMinClusterSize(/*200*/minClusterSize);//设置聚类中点的数量下限，如果点数量少于该值，应用合并算法，合并到最近临的一个聚类

    reg.extract(jlClusters);
}

unsigned int pct::colorstr2int(QString c)
{
    c.remove(' ');
    QStringList l = c.split(',');
    if (l.size() != 3)
    {
        std::cerr << "colorstr2int函数中的color格式错误" << std::endl;
        return 0;
    }
       

    unsigned int colorint = 0;
    *(((unsigned char *)&colorint) + 2) = l[0].toUInt();
    *(((unsigned char *)&colorint) + 1) = l[1].toUInt();
    *(((unsigned char *)&colorint) + 0) = l[2].toUInt();

    return colorint;
}

unsigned int pct::pointsCountsForColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& clusters, const unsigned int color)
{
    unsigned int color_count = 0;
    //char r = color&(0x00ff0000) >> 16;
    unsigned char r = *(((unsigned char *)&color) + 2);
    unsigned char g = *(((unsigned char *)&color) + 1);
    unsigned char b = *(((unsigned char *)&color) + 0);

    //cout << "pointsCountsForColor\t" << "r" << (unsigned int)r << "g" << (unsigned int)g << "b" << (unsigned int)b << std::endl;

    pcl::PointXYZRGB *pt;
    for (auto it = clusters.indices.begin(); it != clusters.indices.end(); ++it)
    {
        pt = &cloud->at(*it);
        //cout << "r" << (unsigned int)pt->r << "g" << (unsigned int)pt->g << "b" << (unsigned int)pt->b << std::endl;
        //cout << "r" << (unsigned int)pt->r + 1 << "g" << (unsigned int)pt->g + 1 << "b" << (unsigned int)pt->b + 1 << std::endl;
        if (
            (pt->r == r || pt->r + 1 == r)
            && (pt->g == g || pt->g + 1 == g)
            && (pt->b == b || pt->b + 1 == b)
            )  // cgal内部用的float存储颜色，有精度丢失
        {
            ++color_count;
        }
    }
    //std::cout << "color_count" << color_count <<  std::endl;
    return color_count;
}

void GetPointsInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices, Vector3 &min, Vector3& max, Vector3& centor, pct::LineInfo::Asix &max_axis)
{
    min.x = std::numeric_limits<float>::max();
    min.y = std::numeric_limits<float>::max();
    min.z = std::numeric_limits<float>::max();
    max.x = -std::numeric_limits<float>::max();
    max.y = -std::numeric_limits<float>::max();
    max.z = -std::numeric_limits<float>::max();

    pcl::PointXYZRGB *pt;
    for (int i = 0; i < indices.indices.size(); ++i)
    {
        pt = &cloud->at(indices.indices[i]);
        if (pt->x < min.x)
            min.x = pt->x;
        if (pt->x > max.x)
            max.x = pt->x;
        if (pt->y < min.y)
            min.y = pt->y;
        if (pt->y > max.y)
            max.y = pt->y;
        if (pt->z < min.z)
            min.z = pt->z;
        if (pt->z > max.z)
            max.z = pt->z;
    }
    float subx = max.x - min.x;
    float suby = max.y - min.y;

    max_axis = subx > suby ? pct::LineInfo::X : pct::LineInfo::Y;

    centor = (max + min) / 2;
}

bool compX(const Vector3 &v1, const Vector3 &v2)
{
    return v1.x < v2.x;
}

bool compY(const Vector3 &v1, const Vector3 &v2)
{
    return v1.y < v2.y;
}

Vector3 GetMaxAxisVec(const pct::LineInfo &info)
{
    Vector3 sub;
    sub = info.end - info.sta;
    //sub.z = 0;
    sub = sub.normalize();
    return sub;
}

double pct::Distance3d(Vector3 &pt1, Vector3 &pt2)
{
    return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2) + pow((pt1.z - pt2.z), 2));
}


double pct::Distance2d(double x, double y, double m, double n)
{
    return  sqrt((x - m)*(x - m) + (y - n)*(y - n));
}

// 获得此根线的特征
pct::LineInfo pct::lineInfoFactory(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices)
{
    LineInfo info;
    info.indices = indices;
    // 装载所有点
    info.pts.clear();
    double block = 1;

    int cloudSize = indices.indices.size();
    info.pts.resize(cloudSize);
    for (int i = 0; i < cloudSize; ++i)
    {
        pcl::PointXYZRGB &pt = cloud->at(indices.indices[i]);
        info.pts[i].x = pt.x;
        info.pts[i].y = pt.y;
        info.pts[i].z = pt.z;
    }

    Vector3 min;
    Vector3 max;
    GetPointsInfo(cloud, indices, min, max, info.center, info.maxAsix);

    std::vector<Vector3> pts = info.pts;
    for (int i = 0; i < cloudSize; ++i)
    {
        pts[i] -= info.center;
    }

    // 初始化拟合变量
    std::vector<float> xDim(cloudSize), yDim(cloudSize), zDim(cloudSize);
    float tmpX = 0, tmpY = 0, tmpZ = 0;

    if (LineInfo::X == info.maxAsix)//x轴跨度大
    {
        std::sort(pts.begin(), pts.end(), compX);

        for (int i = 0; i < cloudSize; ++i)
        {
            xDim[i] = pts[i].x;
            yDim[i] = pts[i].y;
            zDim[i] = pts[i].z;
        }

        info.fit.linearFit(xDim, yDim);

        info.fit_z.polyfit(xDim, zDim, 2);

        info.sta.x = xDim[0];
        info.sta.y = info.fit.getY(xDim[0]);
        info.sta.z = info.fit_z.getY(xDim[0]);

        info.end.x = xDim[cloudSize - 1];
        info.end.y = info.fit.getY(xDim[cloudSize - 1]);
        info.end.z = info.fit_z.getY(xDim[cloudSize - 1]);

        info.v = GetMaxAxisVec(info);
        info.sta += info.center;
        info.end += info.center;

        int fitsize = ceil(Distance3d(info.end, info.sta) / block) + 1;
        info.fit_pts.resize(fitsize);
        info.fit_pts[0] = info.sta;
        for (int i = 1; i < fitsize; ++i)
        {
            info.fit_pts[i] = info.sta + info.v*(block*i);
            info.fit_pts[i].y = info.fit.getY(info.fit_pts[i].x - info.center.x) + info.center.y;
            info.fit_pts[i].z = info.fit_z.getY(info.fit_pts[i].x - info.center.x) + info.center.z;
        }
    }
    else//y轴跨度大
    {
        std::sort(pts.begin(), pts.end(), compY);

        for (int i = 0; i < cloudSize; ++i)
        {
            xDim[i] = pts[i].x;
            yDim[i] = pts[i].y;
            zDim[i] = pts[i].z;
        }

        info.fit.linearFit(yDim, xDim);
        info.fit_z.polyfit(yDim, zDim, 2);

        info.sta.x = info.fit.getY(yDim[0]);
        info.sta.y = yDim[0];
        info.sta.z = info.fit_z.getY(yDim[0]);

        info.end.x = info.fit.getY(yDim[cloudSize - 1]);
        info.end.y = yDim[cloudSize - 1];
        info.end.z = info.fit_z.getY(yDim[cloudSize - 1]);

        info.v = GetMaxAxisVec(info);
        info.sta += info.center;
        info.end += info.center;

        int fitsize = ceil(Distance3d(info.end,info.sta) / block) + 1;
        info.fit_pts.resize(fitsize);
        info.fit_pts[0] = info.sta;
        for (int i = 1; i < fitsize; ++i)
        {
            info.fit_pts[i] = info.sta + info.v*(block*i);
            info.fit_pts[i].x = info.fit.getY(info.fit_pts[i].y - info.center.y) + info.center.x;
            info.fit_pts[i].z = info.fit_z.getY(info.fit_pts[i].y - info.center.y) + info.center.z;
        }
    }

    return info;
}

bool pct::LikePowerLine(pct::LineInfo &line, int min_length /*= 5*/, double error_probability /*= 0.1*/
    , float yerrOffset /*= 1.0f*/, float zerrOffset /*= 0.5f*/)
{
    pct::LineInfo::Asix maxAxis = line.maxAsix;
    float distance = Distance3d(line.sta, line.end);

    if (distance < min_length)
    {
        return false;
    }


    int ptSize = line.pts.size();
    int errpt = 0;
    Vector3 pt;
    for (int i = 0; i < ptSize; ++i)
    {
        pt = line.pts[i] - line.center;
        if (pct::LineInfo::X == maxAxis)
        {
            if (abs(line.fit.getY(pt.x) - pt.y) > yerrOffset || abs(line.fit_z.getY(pt.x) - pt.z) > zerrOffset)
            {
                ++errpt;
            }
        }
        else
        {
            if (abs(line.fit.getY(pt.y) - pt.x) > yerrOffset || abs(line.fit_z.getY(pt.y) - pt.z) > zerrOffset)
            {
                ++errpt;
            }
        }
    }

    std::cout << "疑似电力线：:点数量" << ptSize << "\t 偏离点数量:" << errpt << "\t错误率"  
        << errpt / (double)ptSize << "\t是否是电力线" << (errpt / (double)ptSize > error_probability) << std::endl;
    if (errpt / (double)ptSize > error_probability)
    {
        return false;
    }

    return true;
}

bool pct::LikePowerLine1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud, pct::LineInfo &line, int min_length /*= 5*/, double error_probability /*= 0.1*/
    , float yerrOffset /*= 1.0f*/, float zerrOffset /*= 0.5f*/)
{
    float distance = Distance2d(line.sta.x, line.sta.y, line.end.x, line.end.y);
    pct::LineInfo::Asix maxAxis = line.maxAsix;
    if (LineInfo::Asix::Z == maxAxis || distance < min_length)
    {
        return false;
    }

    pcl::PointXYZRGB maxZ;
    maxZ.z = -std::numeric_limits<float>::max();
    for (int i = 0; i < line.pts.size(); ++i)
    {
        if (line.pts[i].z > maxZ.z)
            maxZ.z = line.pts[i].z;
    }

    pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
    ground_kdtree.setInputCloud(ground_cloud);
    std::vector<int> indices;
    std::vector<float> sqr_distances;

    // 最高点与离地高度<10，则认为不是电力线！
    if (ground_kdtree.radiusSearch(maxZ, 10, indices, sqr_distances) > 0)
    {
        std::cout << "最高点与离地高度<10，则认为不是电力线！" << std::endl;
        return false;
    }

    // 如果小于15米，则判断他偏离率
    if (distance < 15)
    {
        int ptSize = line.pts.size();
        int errpt = 0;
        Vector3 pt;
        for (int i = 0; i < ptSize; ++i)
        {
            pt = line.pts[i] - line.center;
            if (pct::LineInfo::X == maxAxis)
            {
                if (abs(line.fit.getY(pt.x) - pt.y) > yerrOffset || abs(line.fit_z.getY(pt.x) - pt.z) > zerrOffset)
                {
                    ++errpt;
                }
            }
            else
            {
                if (abs(line.fit.getY(pt.y) - pt.x) > yerrOffset || abs(line.fit_z.getY(pt.y) - pt.z) > zerrOffset)
                {
                    ++errpt;
                }
            }
        }

        if (errpt / (double)ptSize > error_probability)
        {
            return false;
        }
    }

    return true;
}

void pct::getMinMax3D(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
    const pcl::PointIndices &indices, pcl::PointXYZRGB &min_pt, pcl::PointXYZRGB &max_pt)
{
    Eigen::Vector4f fmin_pt;
    Eigen::Vector4f fmax_pt;
    pcl::getMinMax3D(cloud, indices, fmin_pt, fmax_pt);
    min_pt.x = fmin_pt.x();
    min_pt.y = fmin_pt.y();
    min_pt.z = fmin_pt.z();

    max_pt.x = fmax_pt.x();
    max_pt.y = fmax_pt.y();
    max_pt.z = fmax_pt.z();
}

void getMinMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> &indices, Vector3 &min, Vector3 &max)
{
    min.x = std::numeric_limits<float>::max();
    max.x = -std::numeric_limits<float>::max();
    min.y = std::numeric_limits<float>::max();
    max.y = -std::numeric_limits<float>::max();
    min.z = std::numeric_limits<float>::max();
    max.z = -std::numeric_limits<float>::max();



    for (int i = 0; i < indices.size(); ++i)
    {
        const float &curX = cloud->at((int)indices[i]).x;
        if (curX >  max.x)
        {
            max.x = curX;
        }
        if (curX < min.x)
        {
            min.x = curX;
        }
        const float &curY = cloud->at((int)indices[i]).y;
        if (curY > max.y)
        {
            max.y = curY;
        }
        if (curY <  min.y)
        {
            min.y = curY;
        }
        const float &curZ = cloud->at((int)indices[i]).z;
        if (curZ > max.z)
        {
            max.z = curZ;
        }
        if (curZ < min.z)
        {
            min.z = curZ;
        }
    }
}

ClusterInfo getClusterInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> &indices)
{
    ClusterInfo info;
    getMinMax(cloud, indices, info.min, info.max);
    info.center = (info.min + info.max) / 2;
    return info;
}

ClusterInfo getClusterInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<int> indices(cloud->size());
    for (int i = 0; i < indices.size(); ++i)
    {
        indices[i] = i;
    }
    ClusterInfo info;
    getMinMax(cloud, indices, info.min, info.max);
    info.center = (info.min + info.max) / 2;
    return info;
}

bool pct::likeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> indices)
{
    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();



    for (int i = 0; i < indices.size(); ++i)
    {
        const float &curX = cloud->at((int)indices[i]).x;
        if (curX > maxX)
        {
            maxX = curX;
        }
        if (curX < minX)
        {
            minX = curX;
        }
        const float &curY = cloud->at((int)indices[i]).y;
        if (curY > maxY)
        {
            maxY = curY;
        }
        if (curY < minY)
        {
            minY = curY;
        }
        const float &curZ = cloud->at((int)indices[i]).z;
        if (curZ > maxZ)
        {
            maxZ = curZ;
        }
        if (curZ < minZ)
        {
            minZ = curZ;
        }
    }

    float subx = maxX - minX;
    float suby = maxY - minY;
    float subz = maxZ - minZ;



    if (subz < subx || subz < suby || subz < 15 )
        return false;


   
    float z_vec_block_height = 2.f;
    std::vector<int> vec_blocks_counts(std::ceil(subz / z_vec_block_height));
    for (int i = 0; i < indices.size(); ++i)
    {
        const float &curZ= cloud->at((int)indices[i]).z;
        int blockindex = (curZ-minZ) / z_vec_block_height;
        blockindex = (std::min<int>)(vec_blocks_counts.size() - 1, blockindex);
        blockindex = (std::max<int>)(0, blockindex);
        vec_blocks_counts[blockindex] ++;
    }

    bool hasgap = false;
    for (int i = 0; i < vec_blocks_counts.size(); ++i)
    {
        if (vec_blocks_counts[i] < 1)
        {
            hasgap = true;
            break;
        }
    }

    if (hasgap)
        return false;

    return true;
}

void delRepeat(std::vector<int>& indices)
{
    std::set<uint> norepeat;

    for (int a = 0; a < indices.size(); ++a)
    {
        norepeat.insert(indices[a]);
    }
    indices.clear();
    indices.insert(indices.end(), norepeat.begin(), norepeat.end());
}

void distanceSerach(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, const std::vector<int>& src_indices,
    double cenX, double cenY, double cenZ, double dis, std::vector<int>& indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(boost::make_shared<std::vector<int>>(src_indices));
    extract.filter(*cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<float> indicesDistance;           //存储近邻点对应平方距离
    pcl::PointXYZRGB pt;
    pt.x = cenX;
    pt.y = cenY;
    pt.z = cenZ;
    kdtree.radiusSearch(pt, dis, indices, indicesDistance);

    for (int i = 0; i < indices.size(); ++i)
    {
        indices[i] = src_indices[indices[i]];
    }
}

void pct::ouShiFenGe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, const std::vector<int> &indeces, std::vector<pcl::PointIndices>& cluster_indices, double k)
{
    int       minClusterSize = 0;
    int       maxClusterSize = 500000000;

    // 讀取文件
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 建立用於提取搜尋方法的kdtree樹物件
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(src_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(k);            
    ec.setMinClusterSize(minClusterSize); 
    ec.setMaxClusterSize(maxClusterSize); 
    ec.setSearchMethod(tree);             
    ec.setInputCloud(src_cloud);
    ec.setIndices(boost::make_shared<std::vector<int>>(indeces));
    ec.extract(cluster_indices);          
}

void pct::ouShiFenGe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<pcl::PointIndices>& cluster_indices, double k)
{
    int       minClusterSize = 0;
    int       maxClusterSize = 500000000;

    // 讀取文件
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 建立用於提取搜尋方法的kdtree樹物件
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(src_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(k);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(src_cloud);
    ec.extract(cluster_indices);
}

// 从铁塔数据中删除离群点，移动到普通点中
void pct::deleteObbErrorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<int> &indeces, std::set<int> &error_points)
{
    double k = 2;
    
    std::vector<pcl::PointIndices> cluster_indices;
    ouShiFenGe(src_cloud, indeces, cluster_indices, k);
    int max_index = -1;
    int max_cluster_size = -1;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        if ((int)cluster_indices[i].indices.size() > max_cluster_size)
        {
            max_index = i;
            max_cluster_size = cluster_indices[i].indices.size();
        }
    }
    indeces = cluster_indices[max_index].indices;


    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        if (max_index != i)
        {
            error_points.insert(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
        }
    }
}

void pct::deleteObbErrorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<int> &indeces, std::vector <pcl::PointIndices> &error_points)
{
    double k = 2;

    std::vector<pcl::PointIndices> cluster_indices;
    pct::ouShiFenGe(src_cloud, indeces, cluster_indices, k);
    int max_index = -1;
    int max_cluster_size = -1;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        if ((int)cluster_indices[i].indices.size() > max_cluster_size)
        {
            max_index = i;
            max_cluster_size = cluster_indices[i].indices.size();
        }
    }
    indeces = cluster_indices[max_index].indices;


    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        if (max_index != i)
        {
            error_points.push_back(cluster_indices[i]);
        }
    }
}


bool pct::DelDir(const QString &path)
{
    if (path.isEmpty()){
        return false;
    }
    QDir dir(path);
    if (!dir.exists()){
        return true;
    }
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //设置过滤
    QFileInfoList fileList = dir.entryInfoList(); // 获取所有的文件信息
    foreach(QFileInfo file, fileList){ //遍历文件信息
        if (file.isFile()){ // 是文件，删除
            file.dir().remove(file.fileName());
        }
        else{ // 递归删除
            DelDir(file.absoluteFilePath());
        }
    }
    return dir.rmpath(dir.absolutePath()); // 删除文件夹
}

void pct::MergeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, 
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& jlClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters)
{
    ClusterInfo cloud_info = getClusterInfo(src_cloud);
    for (int i = 0; i < (int)towerClusters.size() - 1; ++i)
    {
        ClusterInfo infoi = getClusterInfo(src_cloud, towerClusters[i].indices.indices);
        for (int j = i + 1; j < towerClusters.size(); ++j)
        {
            if (!towerClusters[i].indices.indices.size())
                break;
            if (!towerClusters[j].indices.indices.size())
                continue;

            ClusterInfo infoj = getClusterInfo(src_cloud, towerClusters[j].indices.indices);
            double dis = Distance2d(infoi.max.x, infoi.max.y, infoj.max.x, infoj.max.y);
            if (dis < 20 && std::abs(infoi.max.z - infoj.max.z) < 2)
            {
                std::cout << "合并两个铁塔的距离" << dis << "<20" << std::endl;
                towerClusters[j].indices.indices.insert(towerClusters[j].indices.indices.end(), towerClusters[i].indices.indices.begin(), towerClusters[i].indices.indices.end());
                towerClusters[i].indices.indices.clear();
            }
        }
    }
    std::cout << "合并铁塔   条件：距离小于20米，且高度相差不到两米" << std::endl;

    // 删除空的，去除重复的
    for (auto it = towerClusters.begin(); it != towerClusters.end();)
    {
        if ((*it).indices.indices.size() == 0)
            it = towerClusters.erase(it);
        else
        {
            delRepeat(it->indices.indices);
            ++it;
        }
    }

    std::cout << "铁塔个数" << towerClusters.size() << std::endl;


    // 计算铁塔obb， 
    double subz = cloud_info.max.z - cloud_info.min.z;
    std::vector<std::vector < int >> del_vec(towerClusters.size(), std::vector <int >());
#pragma omp parallel for
    for (int i = 0; i < towerClusters.size(); ++i)
    {
        // 初始化obb计算所需要的参数
        int ptct = towerClusters[i].indices.indices.size();
        boost::shared_ptr<vec> points(new vec[ptct], std::default_delete<vec[]>());
        pcl::PointXYZRGB min, max;
        pct::getMinMax3D(*src_cloud, towerClusters[i].indices, min, max);
        double tower_height = max.z - min.z;
        double z_scale = 1;
        if (tower_height < 200)
        {
            z_scale = 200 / tower_height;
        }

        for (int j = 0; j < towerClusters[i].indices.indices.size(); ++j)
        {
            int &curindex = towerClusters[i].indices.indices[j];
            //points.get()[j] = vec(src_cloud->at(curindex).x, src_cloud->at(curindex).y, src_cloud->at(curindex).z) - vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z);
            points.get()[j] = vec(src_cloud->at(curindex).x, src_cloud->at(curindex).y, src_cloud->at(curindex).z + (src_cloud->at(curindex).z - cloud_info.center.z) * z_scale)
                - vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z); // 
        }
        // 计算铁塔obb，设定铁塔最小长宽高范围为5
        OBB obb;
        vec diagonal;
        obb = OBB::BruteEnclosingOBB(points.get(), ptct);
        obb.Scale(obb.pos, vec(1.8, 1.8, 1.2));
        

        diagonal = obb.HalfDiagonal();
        std::cout << obb.r << std::endl;
        for (int j = 0; j < 3; ++j)
        {
            if (obb.r[j] < 5)
                obb.r[j] = 5;
        }
        std::cout << obb.r << std::endl;

        obb.pos += vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z);
        // 在原始点云中，找在obb包围盒之内的点

        std::vector<int> radiuIndices;
        auto tempfunc = [&](std::vector <pcl::PointIndices> &cluster){
            for (int n = 0; n < cluster.size(); ++n)
            {
                distanceSerach(src_cloud, cluster[n].indices, obb.pos.x, obb.pos.y, obb.pos.z, diagonal.Length(), radiuIndices);
                for (int j = 0; j < radiuIndices.size(); ++j)
                {
                    pcl::PointXYZRGB &pt = src_cloud->at(radiuIndices[j]);
                    if (obb.Contains(vec(pt.x, pt.y, pt.z)))
                    {
                        towerClusters[i].indices.indices.push_back(radiuIndices[j]);
                        del_vec[i].push_back(radiuIndices[j]);
                    }
                }
            }
        };
        tempfunc(jlClusters);
        //tempfunc(lineClusters);
        for (int n = 0; n < lineClusters.size(); ++n)
        {
            distanceSerach(src_cloud, lineClusters[n].indices.indices, obb.pos.x, obb.pos.y, obb.pos.z, diagonal.Length(), radiuIndices);
            for (int j = 0; j < radiuIndices.size(); ++j)
            {
                pcl::PointXYZRGB &pt = src_cloud->at(radiuIndices[j]);
                if (obb.Contains(vec(pt.x, pt.y, pt.z)))
                {
                    towerClusters[i].indices.indices.push_back(radiuIndices[j]);
                    del_vec[i].push_back(radiuIndices[j]);
                }
            }
        }
    }
    // 再对包围盒的点聚类，不是最大聚类中的认为是误判点
    std::vector <pcl::PointIndices> obb_err_points;
    for (int i = 0; i < towerClusters.size(); ++i)
    {
        deleteObbErrorPoints(src_cloud, towerClusters[i].indices.indices, obb_err_points);
    }
    jlClusters.insert(jlClusters.end(), obb_err_points.begin(), obb_err_points.end());
    std::cout << "用欧氏空间距离聚类来过滤obb包围盒误判的点" << obb_err_points.size() << std::endl;

    
    // 收集obb包围盒点
    std::set<int> del_set;
    for (int i = 0; i < del_vec.size(); ++i)
    {
        del_set.insert(del_vec[i].begin(), del_vec[i].end());
    }
    std::cout << "收集obb包围盒内的点"  << std::endl;

    // 从原始索引中先删掉铁塔索引
    auto tempfunc = [&](std::vector <pcl::PointIndices> &cluster){
        std::cout << "tempfunc" << &cluster << std::endl;
        for (auto it = cluster.begin(); it != cluster.end(); )
        {
            std::set<int> cloud_indices_set(it->indices.begin(), it->indices.end());
            for (auto itt = cloud_indices_set.begin(); itt != cloud_indices_set.end();)
            {
                if (del_set.find(*itt) != del_set.end())
                {
                    cloud_indices_set.erase(itt++);
                }
                else
                {
                    ++itt;
                }
            }
            if (!cloud_indices_set.size())
            {
                it = cluster.erase(it);
            }
            else
            {
                it->indices.clear();
                it->indices.insert(it->indices.begin(), cloud_indices_set.begin(), cloud_indices_set.end());
                ++it;
            }
        }
    };
    std::cout << " tempfunc(jlClusters);" << &jlClusters <<  std::endl;
    tempfunc(jlClusters);
    std::cout << "tempfunc(lineClusters);" << &lineClusters << std::endl;
    //tempfunc(lineClusters);
    for (auto it = lineClusters.begin(); it != lineClusters.end();)
    {
        std::set<int> cloud_indices_set(it->indices.indices.begin(), it->indices.indices.end());
        for (auto itt = cloud_indices_set.begin(); itt != cloud_indices_set.end();)
        {
            if (del_set.find(*itt) != del_set.end())
            {
                cloud_indices_set.erase(itt++);
            }
            else
            {
                ++itt;
            }
        }
        if (!cloud_indices_set.size())
        {
            it = lineClusters.erase(it);
        }
        else
        {
            it->indices.indices.clear();
            it->indices.indices.insert(it->indices.indices.begin(), cloud_indices_set.begin(), cloud_indices_set.end());
            ++it;
        }
    }
}

void pct::FindLikeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr cloud_indices, 
    std::vector<std::vector<int>>  &clusters, float dbscaneps, int dbscanmin)
{
    struct vec2f {
        float data[2];
        vec2f(float x, float y){ data[0] = x; data[1] = y; };
        float operator[](int idx) const { return data[idx]; }
    };

    // 计算此点云的包围盒信息
    ClusterInfo cloud_info = getClusterInfo(src_cloud, cloud_indices->indices);


    // 初始化密度聚类的数据类型
     auto dbscan = DBSCAN<vec2f, float>();
     auto data = std::vector<vec2f>();
     for (int i = 0; i < cloud_indices->indices.size(); ++i)
     {
         data.push_back(vec2f(src_cloud->at(cloud_indices->indices[i]).x, src_cloud->at(cloud_indices->indices[i]).y));
     }
 
 
     //参数：数据， 维度（二维）， 考虑半径， 聚类最小
     std::cout << "setting.dbscaneps" << dbscaneps << "setting.dbscanmin" << dbscanmin << std::endl;
     dbscan.Run(&data, 2, dbscaneps, dbscanmin);
     //auto noise = dbscan.Noise;
     
     // 密度聚类结果的放入clusters结果集中
     clusters.resize(dbscan.Clusters.size());
     for (int i = 0; i < dbscan.Clusters.size(); ++i)
     {
         clusters[i].insert(clusters[i].end(), dbscan.Clusters[i].begin(), dbscan.Clusters[i].end());
     }
 
     // 结果集指向到真实的下标索引
     for (auto it = dbscan.Noise.begin(); it != dbscan.Noise.end(); ++it)
     {
         *it = cloud_indices->indices[*it];
     }
     for (auto it = clusters.begin(); it != clusters.end(); ++it)
     {
         for (auto itt = it->begin(); itt != it->end(); ++itt)
         {
             *itt = cloud_indices->indices[*itt];
         }
     }
 
    // 找出不像铁塔的聚类，放入Noise数组里
     std::cout << "高密度区域数量" << clusters.size() << std::endl;
     for (auto it = clusters.begin(); it != clusters.end();)
     {
         if (!pct::likeTower(src_cloud, *it))  // 根据长宽高比例，高度，空间间隙等特征初步验证铁塔
         {
             dbscan.Noise.insert(dbscan.Noise.end(), it->begin(), it->end());
             it = clusters.erase(it);
         }
         else
             ++it;
     }
     std::cout << "密度聚类提取铁塔完成" << std::endl;


     // 把Noise里的索引都放回cloud_indices
     cloud_indices->indices.clear();
     for (int i = 0; i < dbscan.Noise.size(); ++i)
     {
         cloud_indices->indices.push_back(dbscan.Noise[i]);
     }

     std::cout << "把Noise里的索引都放回cloud_indices" << std::endl;
    // 合并铁塔   条件：距离小于20米，且高度相差不到两米

     for (int i = 0; i < (int)clusters.size()-1; ++i)
     {
         ClusterInfo infoi = getClusterInfo(src_cloud, clusters[i]);
         for (int j = i + 1; j < clusters.size(); ++j)
         {
             if (!clusters[i].size())
                 break;
             if (!clusters[j].size())
                 continue;

             ClusterInfo infoj = getClusterInfo(src_cloud, clusters[j]);
             double dis = Distance2d(infoi.max.x, infoi.max.y, infoj.max.x, infoj.max.y);
             if (dis < 20 && std::abs(infoi.max.z - infoj.max.z) <2)
             {
                 std::cout << "合并两个铁塔的距离" << dis << "<20" << std::endl;
                 clusters[j].insert(clusters[j].end(), clusters[i].begin(), clusters[i].end());
                 clusters[i].clear();
             }
         }
     }
     std::cout << "合并铁塔   条件：距离小于20米，且高度相差不到两米" << std::endl;
 
     // 删除空的，去除重复的
     for (auto it = clusters.begin(); it != clusters.end();)
     {
         if ((*it).size() == 0 )
             it = clusters.erase(it);
         else
         {
             delRepeat(*it);
             ++it;
         }
     }
 
     std::cout << "铁塔个数" << clusters.size() << std::endl;

 
      // 计算铁塔obb， 

     std::vector<std::vector < int >> del_vec(clusters.size(), std::vector <int >());
#pragma omp parallel for
     for (int i = 0; i < clusters.size(); ++i)
     {
         // 初始化obb计算所需要的参数
         int ptct = clusters[i].size();
         boost::shared_ptr<vec> points(new vec[ptct], std::default_delete<vec[]>());
         for (int j = 0; j < clusters[i].size(); ++j)
         {
             int &curindex = clusters[i][j];
             points.get()[j] = vec(src_cloud->at(curindex).x, src_cloud->at(curindex).y, src_cloud->at(curindex).z) - vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z);
         }
         // 计算铁塔obb，设定铁塔最小长宽高范围为5
         OBB obb;
         vec diagonal;
         obb = OBB::BruteEnclosingOBB(points.get(), ptct);
         obb.Scale(obb.pos, vec(1.8, 1.8, 1.2));
         diagonal = obb.HalfDiagonal();
         for (int j = 0; j < 3; ++j)
         {
             if (obb.r[j] < 5)
                 obb.r[j] = 5;
         }
         obb.pos += vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z);
         // 在原始点云中，找在obb包围盒之内的点
         std::vector<int> radiuIndices;
         distanceSerach(src_cloud, cloud_indices->indices, obb.pos.x, obb.pos.y, obb.pos.z, diagonal.Length(), radiuIndices);
         for (int j = 0; j < radiuIndices.size(); ++j)
         {
             pcl::PointXYZRGB &pt = src_cloud->at(radiuIndices[j]);
             if (obb.Contains(vec(pt.x, pt.y, pt.z)))
             {
                 clusters[i].push_back(radiuIndices[j]);
                 del_vec[i].push_back(radiuIndices[j]);
             }
         }
     }
     std::set<int> del_set;
     for (int i = 0; i < del_vec.size(); ++i)
     {
         del_set.insert(del_vec[i].begin(), del_vec[i].end());
     }
     std::set<int> obb_err_points;
     for (int i = 0; i < clusters.size(); ++i)
     {
         deleteObbErrorPoints(src_cloud, clusters[i], obb_err_points);
     }
     std::cout << "用欧氏空间距离聚类来过滤obb包围盒误判的点" << obb_err_points.size() << std::endl;

     // 从原始索引中先删掉铁塔索引
     std::set<int> cloud_indices_set(cloud_indices->indices.begin(), cloud_indices->indices.end());
     cloud_indices_set.insert(obb_err_points.begin(), obb_err_points.end());
     for (auto it = cloud_indices_set.begin(); it != cloud_indices_set.end(); )
     {
         if (del_set.find(*it) != del_set.end())
         {
             cloud_indices_set.erase(it++);
         }
         else
         {
             ++it;
         }
     }
     cloud_indices->indices.clear();
     cloud_indices->indices.insert(cloud_indices->indices.begin(), cloud_indices_set.begin(), cloud_indices_set.end());
}

double pct::Distance3d(pcl::PointXYZRGB &pt1, pcl::PointXYZRGB &pt2)
{
    return (double)sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
}

double pct::Distance2d(pcl::PointXYZRGB &pt1, pcl::PointXYZRGB &pt2)
{
    return (double)sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

double pct::Distance2d(Vector3 &pt1, pcl::PointXYZRGB &pt2)
{
    return (double)sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

std::string pct::to_utf8(const wchar_t* buffer, int len)
{
    int nChars = ::WideCharToMultiByte(
        CP_UTF8,
        0,
        buffer,
        len,
        NULL,
        0,
        NULL,
        NULL);
    if (nChars == 0)return"";

    string newbuffer;
    newbuffer.resize(nChars);
    ::WideCharToMultiByte(
        CP_UTF8,
        0,
        buffer,
        len,
        const_cast<char*>(newbuffer.c_str()),
        nChars,
        NULL,
        NULL);

    return newbuffer;
}

std::string pct::to_utf8(const std::wstring& str)
{
    return to_utf8(str.c_str(), (int)str.size());
}

std::string pct::WString2String(const std::wstring& ws)
{
    std::string strLocale = setlocale(LC_ALL, "");
    const wchar_t* wchSrc = ws.c_str();
    size_t nDestSize = wcstombs(NULL, wchSrc, 0) + 1;
    char *chDest = new char[nDestSize];
    memset(chDest, 0, nDestSize);
    wcstombs(chDest, wchSrc, nDestSize);
    std::string strResult = chDest;
    delete[]chDest;
    setlocale(LC_ALL, strLocale.c_str());
    return strResult;
}
// string => wstring
std::wstring pct::String2WString(const std::string& s)
{
    std::string strLocale = setlocale(LC_ALL, "");
    const char* chSrc = s.c_str();
    size_t nDestSize = mbstowcs(NULL, chSrc, 0) + 1;
    wchar_t* wchDest = new wchar_t[nDestSize];
    wmemset(wchDest, 0, nDestSize);
    mbstowcs(wchDest, chSrc, nDestSize);
    std::wstring wstrResult = wchDest;
    delete[]wchDest;
    setlocale(LC_ALL, strLocale.c_str());
    return wstrResult;
}