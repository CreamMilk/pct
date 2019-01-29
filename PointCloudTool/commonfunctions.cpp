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
#include <pcl/filters/uniform_sampling.h>   //���Ȳ���
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
#include <pcl/visualization/cloud_viewer.h>   //��cloud_viewerͷ�ļ�����
#include <windows.h>
#include <string.h>
#include "setting.hpp"
#include "pctio.h"
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QAxObject>
#include <QAxWidget>
#include <chrono>
#include "CoorConv.hpp"
#include "GeoCoordinateSystem.h"
#include "Las2Pnts.h"
#include "DangerousDistanceCheck.h"




#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <boost/locale.hpp>
#include <chrono>   
#include <Scene_points_with_normal_item.h> //��
#include <Item_classification_base.h>
#include <Point_set_item_classification.h>
#include <QFileInfo>
#include <QMessageBox>
#include <QAxObject>
#include <QAxWidget>
#include <QDir>

#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "commonfunctions.h"
#include "setting.hpp"
#include "pctio.h"
#include "mydef.h"
#include "DangerousDistanceCheck.h"

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <vtkOutputWindow.h>
#include <QVTKOpenGLWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <QSurfaceFormat>
#include "Las2Pnts.h"


struct ClusterInfo{
    ClusterInfo() :radiu(0){};
    float radiu;
    Vector3 center;
    Vector3 min;
    Vector3 max;
};

/*��str1�ַ����е�һ�γ��ֵ�str2�ַ����滻��str3*/
void replaceFirst(char *str1, char *str2, char *str3)
{
    char *str4 = new char[strlen(str1) + 1];
    char *p;
    strcpy(str4, str1);
    if ((p = strstr(str1, str2)) != NULL)/*pָ��str2��str1�е�һ�γ��ֵ�λ��*/
    {
        while (str1 != p&&str1 != NULL)/*��str1ָ���ƶ���p��λ��*/
        {
            str1++;
        }
        str1[0] = '/0';/*��str1ָ��ָ���ֵ���/0,�Դ����ض�str1,����str2���Ժ�����ݣ�ֻ����str2��ǰ������*/
        strcat(str1, str3);/*��str1��ƴ����str3,�����str1*/
        strcat(str1, strstr(str4, str2) + strlen(str2));/*strstr(str4,str2)��ָ��str2���Ժ������(����str2),strstr(str4,str2)+strlen(str2)���ǽ�ָ����ǰ�ƶ�strlen(str2)λ������str2*/
    }
    delete str4;
}
/*��str1���ֵ����е�str2���滻Ϊstr3*/
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
    //��ȡӦ�ó���Ŀ¼
    char szapipath[MAX_PATH] = { 0 };//��D:\Documents\Downloads\TEST.exe��
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //replace(szapipath, "\\", "/");
    exe_path = szapipath;
    return exe_path;
}

QString pct::GetFileDir(std::string file_path)
{
    return QFileInfo(QString::fromLocal8Bit(file_path.c_str())).absolutePath();
}

std::string pct::GetExeName()
{
    std::string exe_path = "";
    //��ȡӦ�ó���Ŀ¼
    char szapipath[MAX_PATH] = { 0 };//��D:\Documents\Downloads\TEST.exe��
    GetModuleFileNameA(NULL, szapipath, MAX_PATH);
    //��ȡӦ�ó�������
    char szExe[MAX_PATH] = { 0 };//��TEST.exe��
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
    //��ȡӦ�ó���Ŀ¼
    char szapipath[MAX_PATH] = { 0 };//��D:\Documents\Downloads\TEST.exe��
    strcpy_s(szapipath, input.c_str());
    //��ȡӦ�ó�������
    char szExe[MAX_PATH] = { 0 };//��TEST.exe��
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

    size_t xmlsize = xmls.size();  // ����ѵ���ļ�
    std::map<std::string, std::map<std::string, std::vector<int>>> effects;  // label ���� Ч���ȼ�����
    std::string labelname;  // ����forѭ��̫���ˣ� labelname����һ�°ɣ��õ�ʱ��ע�ⲻҪ��ͻ
    int fit_index=-1;
    int fit_num = -1;

    // װ��xml�ļ����ṹ��
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
            for (BOOST_AUTO(pos, label->second.begin()); pos != label->second.end(); ++pos)  //boost�е�auto
            {
                if (pos->first == "feature")
                {
                    std::cout << "name��" << pos->second.get<std::string>("name") << std::endl;
                    std::cout << "effect��" << pos->second.get<std::string>("effect") << std::endl;
                    freture_map[pos->second.get<std::string>("name")] = pos->second.get<std::string>("effect");
                }
            }
            train_xml.labels[labelname] = freture_map;
        }
        train_xml_vec.push_back(train_xml);
    }


    // ����ϲ��ṹ��
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

    // Ч�� 
    for (auto it = effects.begin(); it != effects.end(); ++it)
    {
        labelname = it->first;  // ��ǩ��
        std::map<std::string, std::string> &effect = combinetrain.labels[labelname];  // Ч�����
        for (auto itt = it->second.begin(); itt != it->second.end(); ++itt)
        {
            auto &effect_name = itt->first;  // Ч����
            auto &effect_vec = itt->second;  // Ч������
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

// ��γ��
void pct::LBHtoXYZ(float &xx, float &yy, float &zz)
{
	double x = xx;
	double y = yy;
	double z = zz;
	CGeoCoordinateSystem::LonLat2ECI(xx, yy, zz, x, y, z);
	xx = x;
	yy = y;
	zz = z;
}

// ��γ��
// void pct::LBHtoXYZ(float &xx, float &yy, float &zz)
// {
// 	int l1, l2, b1, b2;
// 	double l3, b3, L, B, H, X, Y, Z, N;
// 
// 	double PI = 3.1415926;//Բ����
// 	double a = 6378137;//���뾶
// 	double f = 1 / 298.257222101;//����
// 	double b = a - f * a;//�̰뾶
// 	double e2 = (a*a - b * b) / (a*a);//eΪ��һƫ����
// 
// 	H = zz;
// 	L = xx/ 180 * PI;
// 	B = yy/ 180 * PI;
// 	N = a / sqrt(1 - e2 * sin(B)*sin(B));
// 	X = (N + H)*cos(B)*cos(L);
// 	Y = (N + H)*cos(B)*sin(L);
// 	Z = (N*(1 - e2) + H)*sin(B);
// 
// 	xx = X;
// 	yy = Y;
// 	zz = Z;
// }

void pct::UTMXY2LatLon(double &x, double &y, int zone, bool southhemi)
{
    CoorConv::WGS84Corr latlon;
    CoorConv::UTMXYToLatLon(x, y, zone, southhemi, latlon);
    x = CoorConv::RadToDeg(latlon.log);
    y = CoorConv::RadToDeg(latlon.lat); 
}

void pct::UTMXY2LatLon(float &x, float &y, int zone, bool southhemi)
{
	CoorConv::WGS84Corr latlon;
	CoorConv::UTMXYToLatLon(x, y, zone, southhemi, latlon);
	x = CoorConv::RadToDeg(latlon.log);
	y = CoorConv::RadToDeg(latlon.lat);
}

void pct::LatLon2UTMXY(double &x, double &y, int zone)
{
    if ((zone < 1) || (zone > 60))
        zone = std::floor((y + 180.0) / 6) + 1;
    x = DegToRad(x);
    y = DegToRad(y);
    CoorConv::UTMCoor xy;
    CoorConv::LatLonToUTMXY(x, y, zone, xy);
    x = xy.x;
    y = xy.y;
}

// �Ƕ�ת����
double rad(double d)
{
    const double PI = 3.1415926535898;
    return d * PI / 180.0;
}

// ����������γ�ȣ�����֮��Ĵ���ֱ�߾���
double pct::getLonDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
    const float EARTH_RADIUS = 6378.137;

    double radLat1 = rad(fLati1);
    double radLat2 = rad(fLati2);
    double a = radLat1 - radLat2;
    double b = rad(fLong1) - rad(fLong2);
    double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1)*cos(radLat2)*pow(sin(b / 2), 2)));
    s = s * EARTH_RADIUS;
    s = (int)(s * 10000000) / 10000;
    return s;
}

void pct::ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inices);
    extract.filter(*out_cloud);
}

pcl::PointXYZRGB pct::GetMiddlePoint(pcl::PointXYZRGB min, pcl::PointXYZRGB max)
{
	pcl::PointXYZRGB mid;
	mid.x = min.x + max.x;
	mid.y = min.y + max.y;
	mid.z = min.z + max.z;
	return mid;
}

void pct::simpleAndOutlierRemoval(std::string inputfile, std::string outputfile, float gridsize, int model)
{
    if (boost::filesystem::exists(boost::filesystem::path(outputfile.c_str())))
        boost::filesystem::remove(boost::filesystem::path(outputfile.c_str()));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pct::io::Load_las(cloud, inputfile);

    if (model == 0)
    {
        //���Ȳ������Ʋ���ȡ�ؼ���      �����²��������Ĵ���
        pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
        uniform_sampling.setInputCloud(cloud);  //�������
        uniform_sampling.setRadiusSearch(gridsize);   //���ð뾶 
        uniform_sampling.filter(*cloud);   //�˲�
    }
    else
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize(gridsize, gridsize, gridsize);
        grid.setInputCloud(cloud);
        grid.filter(*cloud);
    }
    pct::OutlierRemoval(cloud);
    pct::io::save_las(cloud, outputfile);
}

//void pct::OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    // ��Ⱥ��
//    int liqunK = 10;
//    double avgDistance = 10;
//    // �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
//    //����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //�����˲�������
//    sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
//    sor.setMeanK(liqunK);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
//    sor.setStddevMulThresh(avgDistance);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
//    sor.filter(*cloud);                    //�洢
//}

void pct::OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int liqunK /*= 10*/, double avgDistance /*= 10*/, pcl::PointIndicesPtr cloud_indices /*= nullptr*/)
{
    // ��Ⱥ��
	//int liqunK = 10;
	//double avgDistance = 10;
    // �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
    //����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //�����˲�������
    sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
    if (cloud_indices)
         sor.setIndices(cloud_indices);
    sor.setMeanK(liqunK);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
    sor.setStddevMulThresh(avgDistance);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ

    if (cloud_indices)
        sor.filter(cloud_indices->indices);                    //�洢
    else
        sor.filter(*cloud);
}

// �����ƴ����ά����
void PutPointCloud2Arr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr ground_indices, std::vector<std::vector<pcl::PointIndices::Ptr>> &pointsArr,
    int row, int col, pcl::PointXYZRGB min, pcl::PointXYZRGB max, double gridSize)
{
    // �������
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
        omega[i] = omega[i - 1] + hist[i]; //�ۻ��ֲ�����
        mu[i] = mu[i - 1] + i * hist[i];
    }
    double mean = mu[Histogramdim - 1];// �Ҷ�ƽ��ֵ
    double max = 0;
    int k_max = 0;
    for (int k = 1; k < Histogramdim - 1; k++)
    {
        double PA = omega[k]; // A����ռ�ı���
        double PB = 1 - omega[k]; //B����ռ�ı���
        double value = 0;
        if (fabs(PA) > 0.001 && fabs(PB) > 0.001)
        {
            double MA = mu[k] / PA; //A ��ĻҶȾ�ֵ
            double MB = (mean - mu[k]) / PB;//B��ҶȾ�ֵ
            value = PA * (MA - mean) * (MA - mean) + PB * (MB - mean) * (MB - mean);//��䷽��

            if (value > max)
            {
                max = value;
                k_max = k;
            }
        }
    }
    return k_max;
}

void pct::FindGroundIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices, pcl::PointIndicesPtr ground_indices)
{
    // ������ȡ
     pct::Setting setting = pct::Setting::ins();
    
     int windowsize = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<int>("windowsize").value();
     float slope = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<float>("slope").value();
     float minlDistance = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<float>("minlDistance").value();
     float maxlDistance = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<float>("maxlDistance").value();
 
     // RecoverGround
     double gridSize = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<float>("gridSize").value();
     double heightSize = setting.pt.get_child(pct::to_utf8(pct::String2WString("��ȡ����"))).get_optional<float>("heightSize").value();

//     int windowsize = 2;
//     float slope = 1;
//     float minlDistance = 5;
//     float maxlDistance = 30;
// 
//     // RecoverGround
//     double gridSize = 20;
//     double heightSize = 1;


    //	������̬�˲���
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(windowsize);
    pmf.setSlope(slope);
    pmf.setInitialDistance(minlDistance);
    pmf.setMaxDistance(maxlDistance);

    std::cout << "����ȡ����" << std::endl;
    //��ȡ����
    pmf.extract(ground_indices->indices);
    cloud_indices->indices.reserve(cloud->size() - ground_indices->indices.size());
    std::set<int >ground_indices_set(ground_indices->indices.begin(), ground_indices->indices.end());
    for (int i = 0; i < cloud->size(); ++i)
    {
        if (ground_indices_set.find(i) == ground_indices_set.end())
            cloud_indices->indices.push_back(i);
    }

    if (!ground_indices->indices.size())
    {
        std::cout<< ("����Ϊ�գ�����");
    }
    else
    {
        std::cout << "���лָ�" << std::endl;
        pcl::PointIndices::Ptr ground_erase(new pcl::PointIndices);
        std::vector<pcl::PointIndices::Ptr> towerIndies;

        // �����ά���������ֵ
        pcl::PointXYZRGB min, max;
        pct::getMinMax3D(*cloud, *ground_indices, min, max);
        int row = (max.x - min.x) / gridSize + 1;
        int col = (max.y - min.y) / gridSize + 1;

        std::cout << "��������" << cloud->size() << "\t������������� " << ground_indices->indices.size()<< std::endl;
        std::cout << "��ʼ��ƽ������" << (int)max.x << " " << (int)max.y << std::endl;
        // ��ʼ��ƽ������
        std::vector<std::vector<pcl::PointIndices::Ptr>>pointsArr(row, std::vector<pcl::PointIndices::Ptr>(col));
        std::cout << ("������������") << std::endl;
        PutPointCloud2Arr(cloud, ground_indices, pointsArr, row, col, min, max, gridSize);

        std::cout << "����ƽ������" << std::endl;
        // ����ÿһ������
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

                // ׼����������
                pcl::PointCloud<pcl::PointXYZRGB> planeCloud;
                pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
                extract1.setInputCloud(cloud);
                extract1.setIndices(planeIndices);
                extract1.filter(planeCloud);

                // ����������Ϣ
                pcl::PointXYZRGB min, max;
                pcl::getMinMax3D(planeCloud, min, max);
                int heightLevel = (max.z - min.z) / heightSize + 1;
                int planeCloudSize = planeIndices->indices.size();
                double sumzCloud = 0;

                // ͳ�Ƹ߶�ֱ��ͼռ��
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

                // �߶������е��ռ��С��0.7������Ϊ����յ㣬���и߶��Զ���ֵ����
                int histSize = heighthistset.size();
                if (histSize < heightLevel * 0.7)
                {
                    std::vector<double> hist;
                    for each (int var in heighthistset)
                    {
                        hist.push_back((var) / sumzCloud);
                    }

                    double threshold = Otsu(hist) * heightSize + min.z;
                    //std::cout << "�������У�����[%d][%d]����ߵ�=%f����͵�=%f����ֵ%f", i, j, max.z, min.z, threshold;

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

        std::cout << "����ƽ���������" << std::endl;

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
}

std::tuple<unsigned char, unsigned char, unsigned char> GetMidColor(std::tuple<unsigned char, unsigned char, unsigned char>&c1, std::tuple<unsigned char, unsigned char, unsigned char>&c2, double f)
{
	std::tuple<unsigned char, unsigned char, unsigned char> res;
	std::get<0>(res) = std::get<0>(c1) +(std::get<0>(c2)-std::get<0>(c1)) * f;
	std::get<1>(res) = std::get<1>(c1) +(std::get<1>(c2)-std::get<1>(c1)) * f;
	std::get<2>(res) = std::get<2>(c1) +(std::get<2>(c2)-std::get<2>(c1)) * f;
	return res;
}

void pct::ScreenshotHeightColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, QString pic_dir, math::vec *axis_vec)
{
	if (nullptr == axis_vec)
		return;

	std::string hor_pic = (pic_dir + QStringLiteral("\\�߳���ɫ����ͼ.png")).toLocal8Bit().data();
	std::string vec_pic = (pic_dir + QStringLiteral("\\�߳���ɫ����ͼ.png")).toLocal8Bit().data();

	pcl::PointXYZRGB min;
	pcl::PointXYZRGB max;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*src_cloud, *cloud);
	pcl::getMinMax3D(*cloud, min, max);
	vec cen((min.x + max.x) / 2.f, (min.y + max.y) / 2.f, (min.z + max.z) / 2.f);

	double heig = max.z - min.z;
	if (heig == 0)
		return;


	std::vector<std::tuple<unsigned char, unsigned char, unsigned char>> height_colors(10);
	height_colors[0] = std::tuple<unsigned char, unsigned char, unsigned char>(127, 0, 255);
	height_colors[1] = std::tuple<unsigned char, unsigned char, unsigned char>(0, 0, 255);
	height_colors[2] = std::tuple<unsigned char, unsigned char, unsigned char>(0, 127, 255);
	height_colors[3] = std::tuple<unsigned char, unsigned char, unsigned char>(0, 255, 255);
	height_colors[4] = std::tuple<unsigned char, unsigned char, unsigned char>(0, 255, 127);
	height_colors[5] = std::tuple<unsigned char, unsigned char, unsigned char>(0, 255, 0);
	height_colors[6] = std::tuple<unsigned char, unsigned char, unsigned char>(127, 255, 0);
	height_colors[7] = std::tuple<unsigned char, unsigned char, unsigned char>(255, 255, 0);
	height_colors[8] = std::tuple<unsigned char, unsigned char, unsigned char>(255, 127, 0);
	height_colors[9] = std::tuple<unsigned char, unsigned char, unsigned char>(255, 0, 0);

	std::vector<double> heights(10);
	heights[0] = 2;
	heights[1] = 3;
	heights[2] = 4;
	heights[3] = 8;
	heights[4] = 9;
	heights[5] = 10;
	heights[6] = 11;
	heights[7] = 12;
	heights[8] = 13;
	heights[9] = 100;

	for (int i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZRGB &pt = cloud->at(i);
		double subz = pt.z - min.z;
		std::tuple<unsigned char, unsigned char, unsigned char> c;
		if (subz < heights[1])
		{
			c = GetMidColor(height_colors[0], height_colors[1], (subz - heights[0]) / (heights[1] - heights[0]));
 		}
		else if (subz < heights[2])
		{
			c = GetMidColor(height_colors[1], height_colors[2], (subz - heights[1]) / (heights[2] - heights[1]));
			pt.r = std::get<0>(c);
			pt.g = std::get<1>(c);
			pt.b = std::get<2>(c);
		}
		else if (subz < heights[3])
		{
			c = GetMidColor(height_colors[2], height_colors[3], (subz - heights[2]) / (heights[3] - heights[2]));
		}
		else if (subz < heights[4])
		{
			c = GetMidColor(height_colors[3], height_colors[4], (subz - heights[3]) / (heights[4] - heights[3]));

		}
		else if (subz < heights[5])
		{
			c = GetMidColor(height_colors[4], height_colors[5], (subz - heights[4]) / (heights[5] - heights[4]));
		}
		else if (subz < heights[6])
		{
			c = GetMidColor(height_colors[5], height_colors[6], (subz - heights[5]) / (heights[6] - heights[5]));
		}
		else if (subz < heights[7])
		{
			c = GetMidColor(height_colors[6], height_colors[7], (subz - heights[6]) / (heights[7] - heights[6]));
		}
		else if (subz < heights[8])
		{
			c = GetMidColor(height_colors[7], height_colors[8], (subz - heights[7]) / (heights[8] - heights[7]));
			pt.r = std::get<0>(c);
			pt.g = std::get<1>(c);
			pt.b = std::get<2>(c);
		}
		//else if (subz < heights[9])
		//{
			//c = GetMidColor(height_colors[8], height_colors[9], (subz - heights[8]) / (heights[9] - heights[8]));

		//}
		else
		{
			c = GetMidColor(height_colors[8], height_colors[9], (subz - heights[8]) / (heig - heights[8]));
		}
		pt.r = std::get<0>(c);
		pt.g = std::get<1>(c);
		pt.b = std::get<2>(c);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("HeightColor"));

	view->setBackgroundColor(0,0,0);
	view->addPointCloud<pcl::PointXYZRGB>(cloud, "HeightColorCloud");      // no need to add the handler, we use a random handler by default
	view->resetCamera();

	pcl::visualization::Camera camera;
	view->getCameraParameters(camera);
	float cameralen = (vec(camera.focal[0], camera.focal[1], camera.focal[2]) - vec(camera.pos[0], camera.pos[1], camera.pos[2])).Length() * 0.75;

	math::vec horizontal_seevec = axis_vec[1];  // ȫ��ˮƽ�ۿ�����
	horizontal_seevec.z = 0;
	horizontal_seevec = horizontal_seevec*cameralen + cen;
	math::vec vertical_seevec = axis_vec[2].z * cameralen > 0 ? axis_vec[2] : axis_vec[2]*(-1);  // ȫ�ִ�ֱ�ۿ�����
	vertical_seevec = vertical_seevec*cameralen + cen;
	

	// ˮƽ�ۿ�����
	
	view->setCameraPosition(horizontal_seevec.x, horizontal_seevec.y, horizontal_seevec.z, cen.x, cen.y, cen.z, /*axis_vec[2].x, axis_vec[2].y, axis_vec[2].z*/0,0,1);
	view->saveScreenshot(hor_pic);

	// ��ֱ�ۿ�����
	view->setCameraPosition(vertical_seevec.x, vertical_seevec.y, vertical_seevec.z, cen.x, cen.y, cen.z, axis_vec[1].x, axis_vec[1].y,/* axis_vec[1].z*/0);
	view->saveScreenshot(vec_pic);
	view->close();
}

void pct::colorClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector <pcl::PointIndices>& jlClusters)
{
    float distanceThreshold = 0.5;
    float pointColorThreshold = 0;
    float regionColorThreshold = 0;
    int minClusterSize = 5;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;//����������ɫ�����������ָ���Ķ���
    reg.setInputCloud(cloud);//���÷ָ�ԭʼ����
    reg.setSearchMethod(tree);//�����������������������
    reg.setDistanceThreshold(/*10*/distanceThreshold);//���þ�����ֵ��С�ڸ�ֵ����Ϊ�����
    reg.setPointColorThreshold(/*8*/pointColorThreshold);//���õ�֮���ɫ����ֵ��С�ڸ�ֵ����Ϊһ������
    reg.setRegionColorThreshold(/*15*/regionColorThreshold);//���þ���֮���ɫ����ֵ��С�ڸ�ֵ��Ӧ�úϲ��㷨���ϲ�Ϊͬһ������
    reg.setMinClusterSize(/*200*/minClusterSize);//���þ����е���������ޣ�������������ڸ�ֵ��Ӧ�úϲ��㷨���ϲ�������ٵ�һ������

    reg.extract(jlClusters);
}

void pct::colorClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointIndices& inputindices, std::vector <pcl::PointIndices>& jlClusters)
{
    float distanceThreshold = 0.8;
    float pointColorThreshold = 0;
    float regionColorThreshold = 0;
    int minClusterSize = 5;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;//����������ɫ�����������ָ���Ķ���
    reg.setInputCloud(cloud);//���÷ָ�ԭʼ����
    reg.setIndices(boost::make_shared<pcl::PointIndices>(inputindices));
    reg.setSearchMethod(tree);//�����������������������
    reg.setDistanceThreshold(/*10*/distanceThreshold);//���þ�����ֵ��С�ڸ�ֵ����Ϊ�����
    reg.setPointColorThreshold(/*8*/pointColorThreshold);//���õ�֮���ɫ����ֵ��С�ڸ�ֵ����Ϊһ������
    reg.setRegionColorThreshold(/*15*/regionColorThreshold);//���þ���֮���ɫ����ֵ��С�ڸ�ֵ��Ӧ�úϲ��㷨���ϲ�Ϊͬһ������
    reg.setMinClusterSize(/*200*/minClusterSize);//���þ����е���������ޣ�������������ڸ�ֵ��Ӧ�úϲ��㷨���ϲ�������ٵ�һ������

    reg.extract(jlClusters);
}

unsigned int pct::colorstr2int(QString c)
{
    c.remove(' ');
    QStringList l = c.split(',');
    if (l.size() != 3)
    {
        std::cerr << "colorstr2int�����е�color��ʽ����" << std::endl;
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
            )  // cgal�ڲ��õ�float�洢��ɫ���о��ȶ�ʧ
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


void GetPointsInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices, Vector3 &min, Vector3& max, Vector3& centor, pct::VegetInfo::Asix &max_axis)
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

	max_axis = subx > suby ? pct::VegetInfo::X : pct::VegetInfo::Y;

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

Vector3 GetMaxAxisVec(const pct::VegetInfo &info)
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

// ��ô˸��ߵ�����
pct::LineInfo pct::lineInfoFactory(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices)
{
    LineInfo info;
    info.indices = indices;
    // װ�����е�
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

    // ��ʼ����ϱ���
    std::vector<float> xDim(cloudSize), yDim(cloudSize), zDim(cloudSize);
    float tmpX = 0, tmpY = 0, tmpZ = 0;

    if (LineInfo::X == info.maxAsix)//x���ȴ�
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

        //int fitsize = ceil(Distance3d(info.end, info.sta) / block) + 1;
        //info.fit_pts.resize(fitsize);
        //info.fit_pts[0] = info.sta;
        //for (int i = 1; i < fitsize; ++i)
        //{
        //    info.fit_pts[i] = info.sta + info.v*(block*i);
        //    info.fit_pts[i].y = info.fit.getY(info.fit_pts[i].x - info.center.x) + info.center.y;
        //    info.fit_pts[i].z = info.fit_z.getY(info.fit_pts[i].x - info.center.x) + info.center.z;
        //}
    }
    else//y���ȴ�
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

		//int fitsize = ceil(Distance3d(info.end,info.sta) / block) + 1;
		//info.fit_pts.resize(fitsize);
		//info.fit_pts[0] = info.sta;
		//for (int i = 1; i < fitsize; ++i)
		//{
		//    info.fit_pts[i] = info.sta + info.v*(block*i);
		//    info.fit_pts[i].x = info.fit.getY(info.fit_pts[i].y - info.center.y) + info.center.x;
		//    info.fit_pts[i].z = info.fit_z.getY(info.fit_pts[i].y - info.center.y) + info.center.z;
		//}
    }

    return info;
}

pct::VegetInfo pct::vegetInfoFactory(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices& indices)
{
	VegetInfo info(cloud, indices);
	info.indices = indices;
	// װ�����е�
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

	// ��ʼ����ϱ���
	std::vector<float> xDim(cloudSize), yDim(cloudSize), zDim(cloudSize);
	float tmpX = 0, tmpY = 0, tmpZ = 0;

	if (LineInfo::X == info.maxAsix)//x���ȴ�
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
	}
	else//y���ȴ�
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
	}

	return info;
}

void pct::LoadTowers(QString filepath, std::vector <std::tuple<std::string, double, double, double>> &towerClusters)
{
	std::cout << "LoadTowers()" << filepath.toLocal8Bit().data() << std::endl;
	if (!QFile(filepath).exists())
	{
		std::cout << "void LoadTowers()  !QFile(filepath).exists()" << std::endl;
		return;
	}
	HRESULT r = OleInitialize(0);
	if (r != S_OK && r != S_FALSE) {
		qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
	}
	std::cout << "LoadTowers filepath" << filepath.toLocal8Bit().data() << std::endl;
	QAxObject excel("Excel.Application");
	excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
	excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //���ļ�
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt(); //����

	QString Range = "A2:D" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
	QVariant allEnvDataQVariant = allEnvData->property("Value");
	QVariantList allEnvDataList = allEnvDataQVariant.toList();

	std::cout << "LoadTowers intRows" << intRows << std::endl;
	for (int i = 0; i < intRows - 1; i++)
	{
		QVariantList allEnvDataList_i = allEnvDataList[i].toList();

		std::string serial = allEnvDataList_i[0].toString().toLocal8Bit().data();
		double log = allEnvDataList_i[1].toDouble();
		double lat = allEnvDataList_i[2].toDouble();
		double z = allEnvDataList_i[3].toDouble();
		towerClusters.push_back(std::tuple<std::string, double, double, double>(serial, log, lat, z));
	}

	workbook->dynamicCall("Close (Boolean)", false);
	excel.dynamicCall("Quit()");
	OleUninitialize();
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

    std::cout << "���Ƶ����ߣ�:������" << ptSize << "\t ƫ�������:" << errpt << "\t������"  
        << errpt / (double)ptSize << "\t�Ƿ��ǵ�����" << (errpt / (double)ptSize > error_probability) << std::endl;
    if (errpt / (double)ptSize > error_probability)
    {
        return false;
    }

    return true;
}

bool pct::LikePowerLine1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud, pct::LineInfo &line, int min_length /*= 5*/, double error_probability /*= 0.1*/
    , float yerrOffset /*= 1.0f*/, float zerrOffset /*= 0.5f*/)
{
	if (line.indices.indices.size() < 20)
		return false;
    float distance = Distance2d(line.sta.x, line.sta.y, line.end.x, line.end.y);
    pct::LineInfo::Asix maxAxis = line.maxAsix;
    if (LineInfo::Asix::Z == maxAxis || distance < min_length || line.pts.size() < 30)
    {
        return false;
    }

    pcl::PointXYZRGB maxZ;
    maxZ.z = -std::numeric_limits<float>::max();
    for (int i = 0; i < line.pts.size(); ++i)
    {
        if (line.pts[i].z > maxZ.z)
        {
            maxZ.x  = line.pts[i].x;
            maxZ.y = line.pts[i].y;
            maxZ.z = line.pts[i].z;
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
    ground_kdtree.setInputCloud(ground_cloud);
    std::vector<int> indices;
    std::vector<float> sqr_distances;

    // ��ߵ�����ظ߶�<10������Ϊ���ǵ����ߣ�
    if (ground_kdtree.radiusSearch(maxZ, 10, indices, sqr_distances) > 0)
    {
        std::cout << "��ߵ�����ظ߶�<10������Ϊ���ǵ����ߣ�" << std::endl;
        return false;
    }
//     if (ground_kdtree.radiusSearch(minZ, 5, indices, sqr_distances) > 50)
//     {
//         std::cout << "��͵������<5�ĵ㳬��50������Ϊ���ǵ����ߣ�" << std::endl;
//         return false;
//     }

    // ���С��15�ף����ж���ƫ����
    if (distance < 3)
    {
        return false;
    }
    else if (distance < 15)
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


bool pct::LikeVeget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud, pct::VegetInfo &line, double error_probability /*= 0.1*/
	, float yerrOffset /*= 1.0f*/, float zerrOffset /*= 0.5f*/)
{
	// С��20�������˵�
	if (line.indices.indices.size() < 30)
		return false;
	float distance = Distance2d(line.sta.x, line.sta.y, line.end.x, line.end.y);
	pct::VegetInfo::Asix maxAxis = line.maxAsix;

	pcl::PointXYZRGB minZ;
	minZ.z = std::numeric_limits<float>::max();
	for (int i = 0; i < line.pts.size(); ++i)
	{
		if (line.pts[i].z < minZ.z)
		{
			minZ.x = line.pts[i].x;
			minZ.y = line.pts[i].y;
			minZ.z = line.pts[i].z;
		}
	}

	pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
	ground_kdtree.setInputCloud(ground_cloud);
	std::vector<int> indices;
	std::vector<float> sqr_distances;

	// �������ظ߶�>10������Ϊ����ֲ�
	if (ground_kdtree.radiusSearch(minZ, 10, indices, sqr_distances) <= 0)
	{
		std::cout << "�������ظ߶�>10������Ϊ����ֲ�" << std::endl;
		return false;
	}


	// �������10�ף����ж���ƫ����,�Ƿ��ǵ�����
	if (distance > 10)
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

		if (errpt / (double)ptSize < error_probability)
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
    info.radiu = pct::Distance3d(info.min, info.max);
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
    info.radiu = pct::Distance3d(info.min, info.max);
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

    std::vector<float> indicesDistance;           //�洢���ڵ��Ӧƽ������
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

    // �xȡ�ļ�
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // ���������ȡ�ь�������kdtree�����
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

    // �xȡ�ļ�
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // ���������ȡ�ь�������kdtree�����
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

/*
// ������������ɾ����Ⱥ�㣬�ƶ�����ͨ����
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
*/

void pct::deleteObbErrorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, std::vector<int> &indeces, std::vector <pcl::PointIndices> &error_points)
{
    double k = 4;

    std::vector<pcl::PointIndices> cluster_indices;
    pct::ouShiFenGe(src_cloud, indeces, cluster_indices, k);

    if (cluster_indices.size() <= 1)
        return; 

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
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot); //���ù���
    QFileInfoList fileList = dir.entryInfoList(); // ��ȡ���е��ļ���Ϣ
    foreach(QFileInfo file, fileList){ //�����ļ���Ϣ
        if (file.isFile()){ // ���ļ���ɾ��
            file.dir().remove(file.fileName());
        }
        else{ // �ݹ�ɾ��
            DelDir(file.absoluteFilePath());
        }
    }
    return dir.rmpath(dir.absolutePath()); // ɾ���ļ���
}

void pct::MergeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, 
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& jlClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters)
{

    std::cout << "�ϲ�����ǰ����ʶ��������" << towerClusters.size() << std::endl;
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
            double dis = Distance2d(infoi.center.x, infoi.center.y, infoj.center.x, infoj.center.y);
            if (dis < 30 && std::abs(infoi.max.z - infoj.max.z) < 5)
            {
                std::cout << "�ϲ����������ľ���" << dis << "<20" << std::endl;
                towerClusters[j].indices.indices.insert(towerClusters[j].indices.indices.end(), towerClusters[i].indices.indices.begin(), towerClusters[i].indices.indices.end());
                towerClusters[i].indices.indices.clear();
            }
        }
    }
    std::cout << "�ϲ�����   ����������С��20�ף��Ҹ߶���������" << std::endl;

    // ɾ���յģ�ȥ���ظ���
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

    std::cout << "�ϲ�����������ʶ��������" << towerClusters.size() << std::endl;
   


    // ��������obb�� 
    double subz = cloud_info.max.z - cloud_info.min.z;
    std::vector<std::vector < int >> del_vec(towerClusters.size(), std::vector <int >());
#pragma omp parallel for
    for (int i = 0; i < towerClusters.size(); ++i)
    {
        // ��ʼ��obb��������Ҫ�Ĳ���
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
        // ��������obb���趨������С����߷�ΧΪ5
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
        // ��ԭʼ�����У�����obb��Χ��֮�ڵĵ�

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
    // �ٶ԰�Χ�еĵ���࣬�����������е���Ϊ�����е�
    std::vector <pcl::PointIndices> obb_err_points;
    for (int i = 0; i < towerClusters.size(); ++i)
    {
        deleteObbErrorPoints(src_cloud, towerClusters[i].indices.indices, obb_err_points);
    }
    jlClusters.insert(jlClusters.end(), obb_err_points.begin(), obb_err_points.end());
    std::cout << "��ŷ�Ͽռ�������������obb��Χ�����еĵ�" << obb_err_points.size() << std::endl;

    
    // �ռ�obb��Χ�е�
    std::set<int> del_set;
    for (int i = 0; i < del_vec.size(); ++i)
    {
        del_set.insert(del_vec[i].begin(), del_vec[i].end());
    }
    std::cout << "�ռ�obb��Χ���ڵĵ�"  << std::endl;

    // ��ԭʼ��������ɾ����������
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

/*
void pct::FindLikeTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr cloud_indices, 
    std::vector<std::vector<int>>  &clusters, float dbscaneps, int dbscanmin)
{
    struct vec2f {
        float data[2];
        vec2f(float x, float y){ data[0] = x; data[1] = y; };
        float operator[](int idx) const { return data[idx]; }
    };

    // ����˵��Ƶİ�Χ����Ϣ
    ClusterInfo cloud_info = getClusterInfo(src_cloud, cloud_indices->indices);


    // ��ʼ���ܶȾ������������
     auto dbscan = DBSCAN<vec2f, float>();
     auto data = std::vector<vec2f>();
     for (int i = 0; i < cloud_indices->indices.size(); ++i)
     {
         data.push_back(vec2f(src_cloud->at(cloud_indices->indices[i]).x, src_cloud->at(cloud_indices->indices[i]).y));
     }
 
 
     //���������ݣ� ά�ȣ���ά���� ���ǰ뾶�� ������С
     std::cout << "setting.dbscaneps" << dbscaneps << "setting.dbscanmin" << dbscanmin << std::endl;
     dbscan.Run(&data, 2, dbscaneps, dbscanmin);
     //auto noise = dbscan.Noise;
     
     // �ܶȾ������ķ���clusters�������
     clusters.resize(dbscan.Clusters.size());
     for (int i = 0; i < dbscan.Clusters.size(); ++i)
     {
         clusters[i].insert(clusters[i].end(), dbscan.Clusters[i].begin(), dbscan.Clusters[i].end());
     }
 
     // �����ָ����ʵ���±�����
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
 
    // �ҳ����������ľ��࣬����Noise������
     std::cout << "���ܶ���������" << clusters.size() << std::endl;
     for (auto it = clusters.begin(); it != clusters.end();)
     {
         if (!pct::likeTower(src_cloud, *it))  // ���ݳ���߱������߶ȣ��ռ��϶������������֤����
         {
             dbscan.Noise.insert(dbscan.Noise.end(), it->begin(), it->end());
             it = clusters.erase(it);
         }
         else
             ++it;
     }
     std::cout << "�ܶȾ�����ȡ�������" << std::endl;


     // ��Noise����������Ż�cloud_indices
     cloud_indices->indices.clear();
     for (int i = 0; i < dbscan.Noise.size(); ++i)
     {
         cloud_indices->indices.push_back(dbscan.Noise[i]);
     }

     std::cout << "��Noise����������Ż�cloud_indices" << std::endl;
    // �ϲ�����   ����������С��20�ף��Ҹ߶���������

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
                 std::cout << "�ϲ����������ľ���" << dis << "<20" << std::endl;
                 clusters[j].insert(clusters[j].end(), clusters[i].begin(), clusters[i].end());
                 clusters[i].clear();
             }
         }
     }
     std::cout << "�ϲ�����   ����������С��20�ף��Ҹ߶���������" << std::endl;
 
     // ɾ���յģ�ȥ���ظ���
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
 
     std::cout << "��������" << clusters.size() << std::endl;

 
      // ��������obb�� 

     std::vector<std::vector < int >> del_vec(clusters.size(), std::vector <int >());
#pragma omp parallel for
     for (int i = 0; i < clusters.size(); ++i)
     {
         // ��ʼ��obb��������Ҫ�Ĳ���
         int ptct = clusters[i].size();
         boost::shared_ptr<vec> points(new vec[ptct], std::default_delete<vec[]>());
         for (int j = 0; j < clusters[i].size(); ++j)
         {
             int &curindex = clusters[i][j];
             points.get()[j] = vec(src_cloud->at(curindex).x, src_cloud->at(curindex).y, src_cloud->at(curindex).z) - vec(cloud_info.center.x, cloud_info.center.y, cloud_info.center.z);
         }
         // ��������obb���趨������С����߷�ΧΪ5
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
         // ��ԭʼ�����У�����obb��Χ��֮�ڵĵ�
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
     std::cout << "��ŷ�Ͽռ�������������obb��Χ�����еĵ�" << obb_err_points.size() << std::endl;

     // ��ԭʼ��������ɾ����������
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
*/

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

void pct::mergeBalls(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCrashPoint, std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<ClusterInfo> infos(cluster_indices.size());
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        infos[i] = getClusterInfo(allCrashPoint, cluster_indices[i].indices);
    }

    for (int i = 0; i < infos.size()-1; ++i)
    {
        if (!cluster_indices[i].indices.size())
            continue;
        for (int j = i + 1; j < infos.size(); ++j)
        {
            if (!cluster_indices[j].indices.size())
                continue;
            float dis = pct::Distance3d(infos[i].center, infos[j].center);
            // i������j
            if ((infos[i].radiu > dis + infos[j].radiu) || (infos[j].radiu> dis + infos[i].radiu))
            {
                cluster_indices[i].indices.insert(cluster_indices[i].indices.end(), cluster_indices[j].indices.begin(), cluster_indices[j].indices.end());
                cluster_indices[j].indices.clear();
            }
        }
    }

    // ����յ�
    for (auto it = cluster_indices.begin(); it != cluster_indices.end();)
    {
        if (!it->indices.size())
            it = cluster_indices.erase(it);
        else
            ++it;
    }
}

void pct::ConvGeopnts(std::string inputname)
{
	QFileInfo input_fileinfo(QString::fromLocal8Bit(inputname.c_str()));
	QString basename = input_fileinfo.baseName();
	std::string outputdir = (input_fileinfo.absoluteDir().absolutePath() + QStringLiteral("/") + basename).toLocal8Bit().data();
	std::string outputname = outputdir + "_geo.las";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pct::io::Load_las(cloud, inputname);
	pcl::PointXYZRGB * tmp_pt;
	pct::Setting &setting = pct::Setting::ins();

	std::cout << basename.toLocal8Bit().data() << std::endl;
	for (int i = 0; i < cloud->size(); ++i)
	{
		tmp_pt = &cloud->at(i);
		pct::UTMXY2LatLon(tmp_pt->x, tmp_pt->y, setting.zone, setting. southhemi);
		if (0 == i)
		{
			std::cout << (QString::number(tmp_pt->x, 'f', 8) + QStringLiteral(",") + QString::number(tmp_pt->y, 'f', 8)
				+ QStringLiteral(",") + QString::number(tmp_pt->z, 'f', 1)).toLocal8Bit().data() << std::endl;
		}
		pct::LBHtoXYZ(tmp_pt->x, tmp_pt->y, tmp_pt->z);
		if (0 == i)
		{
			std::cout << (QString::number(tmp_pt->x, 'f', 8) + QStringLiteral(",") + QString::number(tmp_pt->y, 'f', 8)
				+ QStringLiteral(",") + QString::number(tmp_pt->z, 'f', 1)).toLocal8Bit().data() << std::endl;
		}

	}
	pct::io::save_las(cloud, outputname);

	std::cout << outputname << std::endl;
	Las2Pnts l;
	l.run(outputname, outputdir.c_str());
	std::cout << outputdir << std::endl;
	auto start = std::chrono::system_clock::now();
	while (!QFile::remove(outputname.c_str()))
	{
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 10)
			break;
	}

	QString errpath = QString::fromLocal8Bit(outputdir.c_str()) + basename + QStringLiteral("_geo_pnts");
	std::cout << errpath.toLocal8Bit().data() << std::endl;
	start = std::chrono::system_clock::now();
	while (!QFile::rename(errpath, QString::fromLocal8Bit(outputdir.c_str())))
	{
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 10)
		{
			
			break;
		}
	}
}



void PositionCorrection(QString tower_excel)
{
	pct::Setting &setting = pct::Setting::ins();
	bool overrideExcel = setting.overrideExcel;         // ����excel
	bool stampcorrectcell = setting.stampcorrectcell;   // ��ʾ�޸�
	std::vector <std::tuple<double, double>> towerClusters;
	LoadTowers(tower_excel, towerClusters);
	QString src_tower_dir = QString::fromLocal8Bit(setting.exceldir.c_str());

	QDir dir(src_tower_dir);
	if (!dir.exists())
	{
		std::cout << "excelĿ¼������!" << std::endl;
		return;
	}

	QStringList filters;
	filters << QString("*.xlsx");
	dir.setFilter(QDir::Files | QDir::NoSymLinks); //�������͹�������ֻΪ�ļ���ʽ
	dir.setNameFilters(filters);
	QFileInfoList list = dir.entryInfoList();
	int file_count = list.count();
	if (file_count <= 0)
	{
		std::cout << "excel�ļ���ĿС��0!" << std::endl;
		return;
	}
	std::cout << "list" << std::endl;
	QStringList string_list;
	for (int i = 0; i < list.count(); i++)
	{
		QFileInfo file_info = list.at(i);
		QString suffix = file_info.suffix();
		if (QString::compare(suffix, QString("xlsx"), Qt::CaseInsensitive) == 0 && !file_info.fileName().count(QStringLiteral("��ƫ")))
		{
			QString absolute_file_path = file_info.absoluteFilePath();
			string_list.append(absolute_file_path);
		}
	}

	std::cout << "string_list" << string_list.size() << std::endl;
	// ����ԭʼexcels
	for (int i = 0; i < string_list.size(); ++i)
	{
		HRESULT r = OleInitialize(0);
		if (r != S_OK && r != S_FALSE) {
			qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
		}
		QAxObject excel("Excel.Application");
		excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
		excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
		QAxObject *workbooks = excel.querySubObject("WorkBooks");
		QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", string_list[i]); //���ļ�
		QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
		QAxObject * usedrange = worksheet->querySubObject("UsedRange");
		QAxObject * rows = usedrange->querySubObject("Rows");
		int intRows = rows->property("Count").toInt(); //����

		QString Range = "A2:I" + QString::number(intRows);
		QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
		QVariant allEnvDataQVariant = allEnvData->property("Value");
		QVariantList allEnvDataList = allEnvDataQVariant.toList();

		// ��������ԭʼexcelÿ��
		std::cout << "intRows" << intRows << string_list[i].toLocal8Bit().data() << std::endl;
		for (int j = 0; j < intRows - 1; j++)
		{
			QVariantList allEnvDataList_j = allEnvDataList[j].toList();
			double j_log = allEnvDataList_j[7].toDouble();
			double j_lat = allEnvDataList_j[8].toDouble();

			// ����ʶ������
			for (int k = 0; k < towerClusters.size(); ++k)
			{
				double offset = pct::getLonDistance(j_log, j_lat, std::get<0>(towerClusters[k]), std::get<1>(towerClusters[k]));
				std::cout << std::fixed << setprecision(7)
					<< j_log << " " << j_lat << "    "
					<< std::get<0>(towerClusters[k]) << " " << std::get<1>(towerClusters[k]) << " " << offset << std::endl;

				if (offset < 30 && offset > 0.2)    //  30����������20����
				{
					if (stampcorrectcell)
					{
						std::cout << "stampcorrectcell" << std::endl;
						worksheet->querySubObject("Cells(int,int)", j + 2, 8)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
						worksheet->querySubObject("Cells(int,int)", j + 2, 9)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
					}
					worksheet->querySubObject("Cells(int,int)", j + 2, 8)->dynamicCall("SetValue(const QVariant&)", std::get<0>(towerClusters[k]));
					worksheet->querySubObject("Cells(int,int)", j + 2, 9)->dynamicCall("SetValue(const QVariant&)", std::get<1>(towerClusters[k]));
				}
			}
		}
		// ��д��ԭ����
		//allEnvData->dynamicCall("SetValue(const QVariant&)", QVariant(allEnvDataList));//�洢�������ݵ� excel ��,��������,�ٶȼ���
		QString write_path = QDir::toNativeSeparators(QFileInfo(string_list[i]).absoluteFilePath());

		if (!overrideExcel)
			write_path = QFileInfo(string_list[i]).absolutePath() + "\\" + QFileInfo(string_list[i]).baseName() + QStringLiteral("��ƫ.xlsx");
		else
			write_path = QFileInfo(string_list[i]).absoluteFilePath();
		if (QFile::exists(write_path))
			QFile::remove(write_path);

		workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(write_path));//������filepath��ע��һ��Ҫ��QDir::toNativeSeparators��·���е�"/"ת��Ϊ"\"����Ȼһ�����治�ˡ�

		workbook->dynamicCall("Close (Boolean)", false);
		excel.dynamicCall("Quit()");
		OleUninitialize();
	}
}

void PositionCorrection(std::vector <pct::TowerInfo>& towerClusters)
{
	pct::Setting &setting = pct::Setting::ins();
	bool overrideExcel = setting.overrideExcel;         // ����excel
	bool stampcorrectcell = setting.stampcorrectcell;   // ��ʾ�޸�
	QString src_tower_dir = QString::fromLocal8Bit(setting.exceldir.c_str());

	QDir dir(src_tower_dir);
	if (!dir.exists())
	{
		std::cout << "excelĿ¼������!" << std::endl;
		return;
	}

	QStringList filters;
	filters << QString("*.xlsx");
	dir.setFilter(QDir::Files | QDir::NoSymLinks); //�������͹�������ֻΪ�ļ���ʽ
	dir.setNameFilters(filters);
	QFileInfoList list = dir.entryInfoList();
	int file_count = list.count();
	if (file_count <= 0)
	{
		std::cout << "excel�ļ���ĿС��0!" << std::endl;
		return;
	}
	std::cout << "list" << std::endl;
	QStringList string_list;
	for (int i = 0; i < list.count(); i++)
	{
		QFileInfo file_info = list.at(i);
		QString suffix = file_info.suffix();
		if (QString::compare(suffix, QString("xlsx"), Qt::CaseInsensitive) == 0 && !file_info.fileName().count(QStringLiteral("��ƫ")))
		{
			QString absolute_file_path = file_info.absoluteFilePath();
			string_list.append(absolute_file_path);
		}
	}

	std::cout << "string_list" << string_list.size() << std::endl;
	// ����ԭʼexcels
	for (int i = 0; i < string_list.size(); ++i)
	{
		HRESULT r = OleInitialize(0);
		if (r != S_OK && r != S_FALSE) {
			qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
		}
		QAxObject excel("Excel.Application");
		excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
		excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
		QAxObject *workbooks = excel.querySubObject("WorkBooks");
		QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", string_list[i]); //���ļ�
		QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
		QAxObject * usedrange = worksheet->querySubObject("UsedRange");
		QAxObject * rows = usedrange->querySubObject("Rows");
		int intRows = rows->property("Count").toInt(); //����

		QString Range = "A2:I" + QString::number(intRows);
		QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
		QVariant allEnvDataQVariant = allEnvData->property("Value");
		QVariantList allEnvDataList = allEnvDataQVariant.toList();

		// ��������ԭʼexcelÿ��
		std::cout << "intRows" << intRows << string_list[i].toLocal8Bit().data() << std::endl;
		for (int j = 0; j < intRows - 1; j++)
		{
			QVariantList allEnvDataList_j = allEnvDataList[j].toList();
			double j_log = allEnvDataList_j[7].toDouble();
			double j_lat = allEnvDataList_j[8].toDouble();

			// ����ʶ������
			for (int k = 0; k < towerClusters.size(); ++k)
			{
				double tower_k_log = towerClusters[k].cen.x;
				double tower_k_lat = towerClusters[k].cen.y;
				pct::UTMXY2LatLon(tower_k_log, tower_k_lat, setting.zone, setting.southhemi);
				double offset = pct::getLonDistance(j_log, j_lat, tower_k_log, tower_k_lat);


				if (offset < 30 && offset > 0.001)    //  30����������20����
				{

					towerClusters[k].tower_no = worksheet->querySubObject("Cells(int,int)", j + 2, 4)->dynamicCall("Value()").toString().toLocal8Bit().data();
					if (stampcorrectcell)
					{
						std::cout << "stampcorrectcell" << std::endl;
						worksheet->querySubObject("Cells(int,int)", j + 2, 8)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
						worksheet->querySubObject("Cells(int,int)", j + 2, 9)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
					}
					worksheet->querySubObject("Cells(int,int)", j + 2, 8)->dynamicCall("SetValue(const QVariant&)", tower_k_log);
					worksheet->querySubObject("Cells(int,int)", j + 2, 9)->dynamicCall("SetValue(const QVariant&)", tower_k_lat);


					std::cout << std::fixed << setprecision(7)
						<< towerClusters[k].tower_no << j_log << " " << j_lat << "    "
						<< tower_k_log << " " << tower_k_lat << " " << offset << std::endl;
				}
			}
		}
		// ��д��ԭ����
		//allEnvData->dynamicCall("SetValue(const QVariant&)", QVariant(allEnvDataList));//�洢�������ݵ� excel ��,��������,�ٶȼ���
		QString write_path = QDir::toNativeSeparators(QFileInfo(string_list[i]).absoluteFilePath());

		if (!overrideExcel)
			write_path = QFileInfo(string_list[i]).absolutePath() + "\\" + QFileInfo(string_list[i]).baseName() + QStringLiteral("��ƫ.xlsx");
		else
			write_path = QFileInfo(string_list[i]).absoluteFilePath();
		if (QFile::exists(write_path))
			QFile::remove(write_path);

		workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(write_path));//������filepath��ע��һ��Ҫ��QDir::toNativeSeparators��·���е�"/"ת��Ϊ"\"����Ȼһ�����治�ˡ�

		workbook->dynamicCall("Close (Boolean)", false);
		excel.dynamicCall("Quit()");
		OleUninitialize();
	}
}

void LoadTowers(QString filepath, std::vector <std::tuple<double, double>> &towerClusters)
{
	std::cout << "LoadTowers()" << filepath.toLocal8Bit().data() << std::endl;
	if (!QFile(filepath).exists())
	{
		std::cout << "void LoadTowers()  !QFile(filepath).exists()" << std::endl;
		return;
	}
	HRESULT r = OleInitialize(0);
	if (r != S_OK && r != S_FALSE) {
		qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
	}
	std::cout << "LoadTowers filepath" << filepath.toLocal8Bit().data() << std::endl;
	QAxObject excel("Excel.Application");
	excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
	excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //���ļ�
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt(); //����

	QString Range = "A2:D" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
	QVariant allEnvDataQVariant = allEnvData->property("Value");
	QVariantList allEnvDataList = allEnvDataQVariant.toList();

	std::cout << "LoadTowers intRows" << intRows << std::endl;
	for (int i = 0; i < intRows - 1; i++)
	{
		QVariantList allEnvDataList_i = allEnvDataList[i].toList();

		double log = allEnvDataList_i[1].toDouble();
		double lat = allEnvDataList_i[2].toDouble();
		towerClusters.push_back(std::tuple<double, double>(log, lat));
	}

	workbook->dynamicCall("Close (Boolean)", false);
	excel.dynamicCall("Quit()");
	OleUninitialize();
}

void SaveTowers(QString filepath, std::vector <pct::TowerInfo> &towerClusters)
{
	pct::Setting &setting = pct::Setting::ins();
	std::cout << "SaveTowers()" << filepath.toLocal8Bit().data() << std::endl;
	if (!filepath.isEmpty()){
		if (QFile::exists(filepath))
		{
			QFile::remove(filepath);
		}

		HRESULT r = OleInitialize(0);
		if (r != S_OK && r != S_FALSE) {
			std::cout << "Qt: Could not initialize OLE;" << std::endl;
		}
		QAxObject *excel = new QAxObject("Excel.Application");//����Excel�ؼ�
		excel->dynamicCall("SetVisible (bool Visible)", false);//����ʾ����
		excel->setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ�����Ϊtrue��ô�ڹر��ǻ�������ơ��ļ����޸ģ��Ƿ񱣴桱����ʾ
		QAxObject *workbooks = excel->querySubObject("WorkBooks");//��ȡ����������
		workbooks->dynamicCall("Add");//�½�һ��������
		QAxObject *workbook = excel->querySubObject("ActiveWorkBook");//��ȡ��ǰ������
		QAxObject *worksheets = workbook->querySubObject("Sheets");//��ȡ��������
		QAxObject *worksheet = worksheets->querySubObject("Item(int)", 1);//��ȡ�������ϵĹ�����1����sheet1

		QList<QVariant> allRowsData;//��������������
		QList<QVariant> headRowData;//excelͷ

		headRowData.append(QVariant(QStringLiteral("���")));
		headRowData.append(QVariant(QStringLiteral("����")));
		headRowData.append(QVariant(QStringLiteral("γ��")));
		headRowData.append(QVariant(QStringLiteral("�߳�ֵ")));
		allRowsData.append(QVariant(headRowData));
		pct::TowerInfo *tower_iter;

		for (int row = 1; row <= towerClusters.size(); row++)
		{
			tower_iter = &towerClusters[row - 1];
			double x = tower_iter->cen.x;
			double y = tower_iter->cen.y;
			QList<QVariant> aRowData;//����һ������
			pct::UTMXY2LatLon(x, y, setting.zone, setting.southhemi);

			aRowData.append(QVariant(QString::fromLocal8Bit(tower_iter->tower_no.c_str())));
			aRowData.append(QVariant(x));
			aRowData.append(QVariant(y));
			aRowData.append(QVariant(tower_iter->max.z));
			allRowsData.append(QVariant(aRowData));
		}

		QAxObject *range = worksheet->querySubObject("Range(const QString )", QString("A1:D") + QString::number(towerClusters.size() + 1));
		range->dynamicCall("SetValue(const QVariant&)", QVariant(allRowsData));//�洢�������ݵ� excel ��,��������,�ٶȼ���
		range->setProperty("HorizontalAlignment", -4108);
		range->setProperty("VerticalAlignment", -4108);
		QAxObject * cells = range->querySubObject("Columns");
		cells->dynamicCall("AutoFit");

		range->querySubObject("Font")->setProperty("Size", 14);//�����ֺ�

		std::cout << QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()).toLocal8Bit().data() << std::endl;
		workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()));//������filepath��ע��һ��Ҫ��QDir::toNativeSeparators��·���е�"/"ת��Ϊ"\"����Ȼһ�����治�ˡ�
		workbook->dynamicCall("Close (Boolean)", false);
		excel->dynamicCall("Quit()");//�ر�excel
		delete excel;
		excel = NULL;
		OleUninitialize();
	}

}

void SaveLines(QString filepath, std::vector <pct::LineInfo> &lineClusters)
{
	pct::Setting &setting = pct::Setting::ins();
	std::cout << "SaveLines()" << filepath.toLocal8Bit().data() << std::endl;
	if (!filepath.isEmpty()){
		if (QFile::exists(filepath))
			QFile::remove(filepath);
		HRESULT r = OleInitialize(0);
		if (r != S_OK && r != S_FALSE) {
			qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
		}
		QAxObject *excel = new QAxObject("Excel.Application");//����Excel�ؼ�
		excel->dynamicCall("SetVisible (bool Visible)", false);//����ʾ����
		excel->setProperty("DisplayAlerts", true);//����ʾ�κξ�����Ϣ�����Ϊtrue��ô�ڹر��ǻ�������ơ��ļ����޸ģ��Ƿ񱣴桱����ʾ
		QAxObject *workbooks = excel->querySubObject("WorkBooks");//��ȡ����������
		workbooks->dynamicCall("Add");//�½�һ��������
		QAxObject *workbook = excel->querySubObject("ActiveWorkBook");//��ȡ��ǰ������
		QAxObject *worksheets = workbook->querySubObject("Sheets");//��ȡ��������
		QAxObject *worksheet = worksheets->querySubObject("Item(int)", 1);//��ȡ�������ϵĹ�����1����sheet1

		QList<QVariant> allRowsData;//��������������
		QList<QVariant> headRowData;//excelͷ

		headRowData.append(QVariant(QStringLiteral("���")));
		headRowData.append(QVariant(QStringLiteral("��ʼ����")));
		headRowData.append(QVariant(QStringLiteral("��ʼγ��")));
		headRowData.append(QVariant(QStringLiteral("��ʼ�߳�")));
		headRowData.append(QVariant(QStringLiteral("�յ㾭��")));
		headRowData.append(QVariant(QStringLiteral("�յ�γ��")));
		headRowData.append(QVariant(QStringLiteral("�յ�߳�")));
		headRowData.append(QVariant(QStringLiteral("ˮƽ����")));
		allRowsData.append(QVariant(headRowData));
		pct::LineInfo *line_iter;

		for (int row = 1; row <= lineClusters.size(); row++)
		{
			line_iter = &lineClusters[row - 1];
			double stax = line_iter->sta.x;
			double stay = line_iter->sta.y;
			double endx = line_iter->end.x;
			double endy = line_iter->end.y;
			QList<QVariant> aRowData;//����һ������
			pct::UTMXY2LatLon(stax, stay, setting.zone, setting.southhemi);
			pct::UTMXY2LatLon(endx, endy, setting.zone, setting.southhemi);

			aRowData.append(QVariant(QString::fromLocal8Bit(line_iter->getLineNo().c_str()).remove('#').replace('-', '_')));
			aRowData.append(QVariant(stax));
			aRowData.append(QVariant(stay));
			aRowData.append(QVariant(line_iter->sta.z));
			aRowData.append(QVariant(endx));
			aRowData.append(QVariant(endy));
			aRowData.append(QVariant(line_iter->end.z));
			aRowData.append(QVariant(pct::Distance2d(line_iter->sta.x, line_iter->sta.y, line_iter->end.x, line_iter->end.y)));
			allRowsData.append(QVariant(aRowData));
		}

		QAxObject *range = worksheet->querySubObject("Range(const QString )", QString("A1:H") + QString::number(lineClusters.size() + 1));
		range->dynamicCall("SetValue(const QVariant&)", QVariant(allRowsData));//�洢�������ݵ� excel ��,��������,�ٶȼ���
		range->setProperty("HorizontalAlignment", -4108);
		range->setProperty("VerticalAlignment", -4108);
		QAxObject * cells = range->querySubObject("Columns");
		cells->dynamicCall("AutoFit");

		range->querySubObject("Font")->setProperty("Size", 14);//�����ֺ�

		std::cout << QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()).toLocal8Bit().data() << std::endl;
		workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()));//������filepath��ע��һ��Ҫ��QDir::toNativeSeparators��·���е�"/"ת��Ϊ"\"����Ȼһ�����治�ˡ�
		workbooks->dynamicCall("Close (boolean)", false);
		excel->dynamicCall("Quit()");//�ر�excel
		delete excel;
		excel = NULL;
		OleUninitialize();
	}
}

void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
	pcl::PointIndicesPtr ground_indices,
	std::vector <pct::VegetInfo>& vegetClusters,
	std::vector <pct::LineInfo>& lineClusters,
	std::vector <pct::TowerInfo>& towerClusters)
{
	std::cout << "DangerousDistanceCheck" << std::endl;
	DangerousDistanceCheck ddc;
	ddc.setData(src_cloud
		, ground_indices
		, vegetClusters
		, lineClusters
		, towerClusters
		);
	ddc.TooNearCheck();
	ddc.showNearCheck();
	std::cout << "showNearCheck endl." << std::endl;
}

bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
	setsettingitem(classdir, std::string, true, "");
	if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //����ļ��в����ڣ����߲����ļ���
	{
		std::cout << "ѡ��classdir�ļ���·������" << std::endl;
		std::cout << classdir << std::endl;
		return false;
	}
	setsettingitem(gridsize, float, false, 0.3);

	return true;
}

bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
	setsettingitem(inputfile, std::string, true, "");
	boost::filesystem::path path_file(inputfile);
	boost::filesystem::path path_dir(path_file.parent_path().string());

	if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file) || boost::filesystem::extension(path_file) != ".las")
	{
		std::cout << "ѡ��inputfile�ļ�·������" << std::endl;
		std::cout << inputfile << std::endl;
		return false;
	}

	setsettingitem(outputdir, std::string, false, path_dir.string() + "\\" + path_file.stem().string());
	boost::filesystem::path out_dir = boost::filesystem::path(outputdir);

	auto start = std::chrono::system_clock::now();
	while (!pct::DelDir(QString::fromLocal8Bit(outputdir.c_str())))
	{
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 10)
		{
			break;
		}
	}
	if (!boost::filesystem::exists(out_dir))
	{
		boost::filesystem::create_directories(out_dir);
	}
	if (!boost::filesystem::is_directory(out_dir))
	{
		std::cout << "ѡ��outputdirĿ¼����" << std::endl;
		std::cout << outputdir << std::endl;
		return false;
	}

	setsettingitem(classdir, std::string, true, "");
	if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //����ļ��в����ڣ����߲����ļ���
	{
		std::cout << "�����ļ�����û��ѵ���ļ�config.xml������ѵ��������" << std::endl;
		std::cout << classdir << std::endl;
		return false;
	}
	setsettingitem(method, int, false, 2);

	setsettingitem(gridsize, float, false, 0.3);

	setsettingitem(zone, int, true, 50);

	setsettingitem(southhemi, bool, false, false);

	return true;
}

bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
	if (!ReadyClassifOpts(setting, vm))
	{
		return false;
	}

	return true;
}

bool ReadPoscorrectOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
	std::cout << "ReadPoscorrectOpts" << std::endl;
	setsettingitem(inputfile, std::string, true, "");

	boost::filesystem::path path_file(inputfile);
	boost::filesystem::path path_dir(path_file.parent_path().string());

	std::cout << "boost::filesystem::extension(path_file)" << std::endl;
	if (boost::filesystem::extension(path_file) == ".xlsx")
	{
		setting.reclassif = false;
	}
	else if (boost::filesystem::extension(path_file) == ".las")
	{
		setting.reclassif = true;
	}
	else
	{
		std::cout << "ѡ��inputfile�ļ�·������" << std::endl;
		std::cout << inputfile << std::endl;
		return false;
	}
	std::cout << "setting.reclassif" << setting.reclassif << std::endl;

	if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
	{
		std::cout << "ѡ��inputfile�ļ�·������" << std::endl;
		std::cout << inputfile << std::endl;
		return false;
	}

	std::cout << "path_dir.string()" << path_dir.string() + "\\" + path_file.stem().string() << std::endl;
	if (setting.reclassif == true)
	{
		setsettingitem(outputdir, std::string, false, path_dir.string() + "\\" + path_file.stem().string());
		boost::filesystem::path out_dir = boost::filesystem::path(outputdir);


		if (!boost::filesystem::exists(out_dir))
		{
			boost::filesystem::create_directories(out_dir);
		}
		if (!boost::filesystem::is_directory(out_dir))
		{
			std::cout << "ѡ��outputdirĿ¼����" << std::endl;
			std::cout << outputdir << std::endl;
			return false;
		}
	}
	else
	{
		setting.outputdir = path_dir.string();
	}

	setsettingitem(exceldir, std::string, true, "");
	boost::filesystem::path excel_dir = boost::filesystem::path(exceldir);
	if (!boost::filesystem::exists(excel_dir) || !boost::filesystem::is_directory(excel_dir))
	{
		std::cout << "ѡ��excel_dirĿ¼����" << std::endl;
		std::cout << "excel_dir" << excel_dir << std::endl;
		return false;
	}

	setsettingitem(classdir, std::string, true, "");
	if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //����ļ��в����ڣ����߲����ļ���
	{
		std::cout << "�����ļ�����û��ѵ���ļ�config.xml������ѵ��������" << std::endl;
		std::cout << classdir << std::endl;
		return false;
	}
	setsettingitem(method, int, false, 2);

	setsettingitem(gridsize, float, false, 0.3);

	//setsettingitem(reclassif, bool, false, true);

	setsettingitem(overrideExcel, bool, false, false)

		setsettingitem(stampcorrectcell, bool, false, true);

	setsettingitem(zone, int, true, 50);

	setsettingitem(southhemi, bool, false, false);

	return true;
}

bool ParserCmdline(int argc, char *argv[])
{
	std::cout << "ParserCmdline entry" << std::endl;
	std::string exe_name = pct::GetExeName();
	pct::Setting& setting = pct::Setting::ins();
	// ����������
	boost::program_options::variables_map vm;
	boost::program_options::options_description opts("pointcloud tool options"
		"\nʾ����"
		"\n��1��" + exe_name + " --cmdtype train --classdir classdir --gridsize 0.3"
		"\n��2��" + exe_name + " --cmdtype classif --inputfile inputfile --outputdir outputdir --classdir classdir --method 2 --zone 50 --southhemi false"
		"\n��3��" + exe_name + " --cmdtype distancecheck --inputfile inputfile --outputdir outputdir --classdir classdir --method 2 --zone 50 --southhemi false"
		"\n��4��" + exe_name + " --cmdtype poscorrect --inputfile inputfile --overrideExcel false --stampcorrectcell true  --exceldir \"������ƫ����\"  --outputdir outputdir --classdir classdir --method 2 --zone 50 --southhemi false"
		"\n����");

	opts.add_options()
		("cmdtype", boost::program_options::value<std::string>(), "�������ͣ�train classif distancecheck��")
		("inputfile", boost::program_options::value<std::string>(), "����·����*.las��")
		("classdir", boost::program_options::value<std::string>(), "�����ļ�Ŀ¼ <dir>")
		("outputdir", boost::program_options::value<std::string>(), "������Ŀ¼ [dir]")
		("method", boost::program_options::value<int>(), "���෽�� [int] 0-��ͨ 1-ƽ�� 2-�ǳ�ƽ��")
		("gridsize", boost::program_options::value<float>(), "Ҷ�ڵ�ߴ� [float]")
		//("reclassif", boost::program_options::value<bool>(), "�Ƿ����·��� [bool]")
		("exceldir", boost::program_options::value<std::string>(), "��ƫ������Ŀ¼ [std::string]")
		("overrideExcel", boost::program_options::value<bool>(), "�Ƿ񸲸�Դexcel [bool]")
		("stampcorrectcell", boost::program_options::value<bool>(), "�Ƿ����޸�cell [bool]")
		("zone", boost::program_options::value<int>(), "������� [int] ��������Ϊ50")
		("southhemi", boost::program_options::value<bool>(), "�Ƿ�Ϊ���ϰ��� [bool]")
		("help", "����");

	try{
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
	}
	catch (...){
		std::cout << "����Ĳ����д���δ�����ѡ�\n";
		return false;
	}

	// ��鲢��ֵ
	if (vm.count("help")){//����������helpѡ��
		std::cout << opts << std::endl;
		return false;
	}
	else
	{
		setsettingitem(cmdtype, std::string, true, "");
		if (cmdtype == "train")
		{
			if (!ReadyTrainOpts(setting, vm))
				return false;
		}
		else if (cmdtype == "classif")
		{
			if (!ReadyClassifOpts(setting, vm))
				return false;
		}
		else if (cmdtype == "distancecheck")
		{
			if (!ReadyDistancecheckOpts(setting, vm))
				return false;
		}
		else if (cmdtype == "poscorrect")
		{
			std::cout << "ReadPoscorrectOpts start" << std::endl;
			if (!ReadPoscorrectOpts(setting, vm))
				return false;
		}
		else
		{
			std::cout << "����cmdtype�������" << std::endl;
			return false;
		}
	}

	return true;
}

bool train()
{
	std::cout << "����ѵ��..." << std::endl;
	const pct::Setting & setting = pct::Setting::ins();
	int nb_scales = setting.value<int>("nb_scales");
	int nb_trials = setting.value<int>("nb_trials");
	int simplify_model = setting.value<int>("simplify");
	// ���������ļ���
	boost::filesystem::path yangbendir(setting.classdir);
	std::cout << "setting.classdir��" << setting.classdir << std::endl;
	boost::filesystem::directory_iterator iter_dirend;

	std::vector<std::string> labels;
	std::vector<std::string> labelxmls;
	for (boost::filesystem::directory_iterator yangbendiriter(yangbendir); yangbendiriter != iter_dirend; ++yangbendiriter)
	{
		// ���������±ߵ����������ļ�
		if (boost::filesystem::is_directory(*yangbendiriter))
		{
			std::vector<std::string> lasxmls;
			std::string classdir = yangbendiriter->path().string(); // �õ����ļ���·��
			std::string classname = yangbendiriter->path().stem().string();  // �õ�����
			std::string labelxml = classdir + "\\config.xml";
			std::cout << "��ʼѵ��" << classname << "..." << std::endl;
			for (boost::filesystem::directory_iterator fileiter(*yangbendiriter); fileiter != iter_dirend; ++fileiter)
			{
				// ����ļ���Ч�����Һ�׺��Ϊlas����������Ҫѵ���Ķ���
				if (boost::filesystem::is_regular_file(*fileiter) && boost::algorithm::to_lower_copy(boost::filesystem::extension(*fileiter)) == ".las")
				{
					std::string laspath = fileiter->path().string(); // �õ��ļ�·��
					std::string lasname = fileiter->path().stem().string(); // �õ��ļ���

					// ��ϡ  
					std::ostringstream tempfile;
					tempfile << setting.outputdir << "\\" << lasname << "_simple.las";
					if (boost::filesystem::exists(boost::filesystem::path(tempfile.str())))
						boost::filesystem::remove(boost::filesystem::path(tempfile.str()));


					pct::simpleAndOutlierRemoval(laspath, tempfile.str(), setting.gridsize, simplify_model);
					boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));


					std::cout << "�ļ���" << lasname << "..." << std::endl;
					if (scene_item && scene_item->point_set()->check_colors())
					{
						auto start = std::chrono::system_clock::now();
						Point_set* points = scene_item->point_set();
						// ��������
						boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
						classif->compute_features(nb_scales);

						// ���label
						classif->add_new_label(classname.c_str());
						classif->add_new_label(unselect_str);

						// ѡ���ɫ��
						classif->change_color(0);
						std::vector<Point_set::Index> unselected, selected;
						for (Point_set::Index idx : *points)
						{
							if (points->red(idx) == 1. && scene_item->point_set()->green(idx) == 0. && scene_item->point_set()->blue(idx) == 0.)
								selected.push_back(idx);
							else
								unselected.push_back(idx);
						}

						// ѡ��,����label
						scene_item->selectPoints(selected, unselected);
						std::cout << "selected red points�� " << points->number_of_removed_points() << std::endl;
						classif->add_selection_to_training_set(0);

						// ��ѡ��,����unselectlabel
						scene_item->selectPoints(unselected, selected);
						//scene_item->invertSelection();  // �м�add_selection_to_training_set��ɾ��ѡ�񼯣�����������������ﲻ����
						std::cout << "unselect points�� " << points->number_of_removed_points() << std::endl;
						classif->add_selection_to_training_set(1);

						// ѵ��
						std::cout << "ѵ��" << classname << "-" << lasname << std::endl;
						classif->train(0, nb_trials, 0, 0);
						std::string lasxml_path = classdir + "\\" + lasname + ".xml";
						classif->save_config(lasxml_path.c_str(), 0);
						lasxmls.push_back(lasxml_path);

						std::cout << "ѵ��" << classname << "��ɡ�һ��" << points->size() << "����,ѡ��" << selected.size() << "����\n��ʱ"
							<< std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() << "s" << std::endl;
					}
				}
			}

			if (lasxmls.size())
			{
				std::cout << lasxmls.size() << std::endl;
				pct::combineTrainXmlFiles(lasxmls, labelxml);
				labelxmls.push_back(labelxml);
			}
		}
	}

	std::cout << labelxmls.size() << std::endl;
	if (labelxmls.size())
		pct::combineTrainXmlFiles(labelxmls, setting.classdir + "\\config.xml");
	return true;
}

bool classif()
{
	std::cout << "��ʼ����..." << std::endl;
	const pct::Setting & setting = pct::Setting::ins();
	int nb_scales = setting.value<int>("nb_scales");
	int method = setting.method;
	std::cout << "method" << method << std::endl;

	std::string labelname_traverse;
	std::string config_xml = setting.classdir + "\\config.xml";

	boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(setting.outputdir + "\\simple.las"));
	if (scene_item)
	{
		// ��������
		boost::shared_ptr<Point_set_item_classification> classifyy(new Point_set_item_classification(scene_item.get()));
		classifyy->compute_features(nb_scales);

		// ���label
		boost::property_tree::ptree pt;
		boost::property_tree::xml_parser::read_xml(config_xml, pt);
		BOOST_AUTO(labels, pt.get_child("classification.labels"));
		for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
		{
			labelname_traverse = label->second.get<std::string>("name");
			classifyy->add_new_label(labelname_traverse.c_str(), setting.cls_intcolor(labelname_traverse));
		}

		// ����
		classifyy->load_config(config_xml.c_str(), 0);
		classifyy->run(method, 0, 16, 0.5);
		pct::io::lassave(scene_item.get(), setting.outputdir + "\\classif.las");
	}
	return true;
}


void ExtractGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
	pcl::PointIndicesPtr cloud_indices,
	pcl::PointIndicesPtr ground_indices)
{
	// ��ȡ���������
	pct::FindGroundIndices(src_cloud, cloud_indices, ground_indices);
	std::cout << "�ǵ���㣺" << cloud_indices->indices.size()
		<< "����㣺" << ground_indices->indices.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(ground_indices);
	extract.filter(*ground);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr FindGroundIndices_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> ground_extract_indices;
	ground_extract_indices.setInputCloud(src_cloud);
	ground_extract_indices.setIndices(ground_indices);
	ground_extract_indices.setNegative(true);
	ground_extract_indices.filter(*FindGroundIndices_cloud);
	for (int i = 0; i < FindGroundIndices_cloud->size(); ++i)
	{
		FindGroundIndices_cloud->at(i).rgb = 0;
	}
	pct::io::save_las(FindGroundIndices_cloud, pct::Setting::ins().outputdir + "\\NoGround.las");
	FindGroundIndices_cloud->clear();
}

void ExtractLinesAndTower(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
	pcl::PointIndicesPtr cloud_indices,
	pcl::PointIndicesPtr ground_indices,
	std::vector <pcl::PointIndices>& jlClusters,
	std::vector <pct::LineInfo>& lineClusters,
	std::vector <pct::TowerInfo>& towerClusters,
	std::vector <pct::VegetInfo> &vegetClusters)
{
	std::cout << "ExtractLinesAndTower begin" << std::endl;

	const pct::Setting & setting = pct::Setting::ins();
	std::string outputfile = setting.outputdir + "\\out.las";
	const int tower_intersectline_threshold = 3;

	// �����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "ExtractCloud" << src_cloud->size() << " " << ground_indices->indices.size() << std::endl;
	pct::ExtractCloud(src_cloud, ground_indices, ground);

	std::cout << "colorClusters begin" << std::endl;
	// �ٶ�ʣ��ĵ���ɫ����
	pct::colorClusters(src_cloud, *cloud_indices, jlClusters);
	std::cout << "����������" << jlClusters.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*src_cloud, *tmpcloud);
	pcl::PointXYZRGB *tmppt1;

	for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
	{
		unsigned char r = rand() % 256;
		unsigned char g = rand() % 256;
		unsigned char b = rand() % 256;

		for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
		{
			tmppt1 = &tmpcloud->at(*itt);
			tmppt1->r = r;
			tmppt1->g = g;
			tmppt1->b = b;
		}
	}
	for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
	{
		tmppt1 = &tmpcloud->at(*it);
		tmppt1->r = 0;
		tmppt1->g = 0;
		tmppt1->b = 255;
	}
	pct::io::save_las(tmpcloud, setting.outputdir + "\\jl.las");
	tmpcloud.reset();

	std::cout << "��������ȡbegin" << std::endl;
	// ��������ȡ
	// �������Ƿ���70%���ϵĵ����ߵ㣬����ǣ���˵���ǵ����ߵ�
	for (auto it = jlClusters.begin(); it != jlClusters.end();)
	{
		std::cout << "��������ȡ1" << std::endl;
		if (pct::pointsCountsForColor(src_cloud, *it, setting.cls_intcolor(power_line_str)) > it->indices.size()*0.25)  // power_line":"255, 255, 0
		{
			std::cout << "��������ȡ2" << std::endl;
			pct::LineInfo line = pct::lineInfoFactory(src_cloud, *it);
			std::cout << "��������ȡbegin" << std::endl;
			if (pct::LikePowerLine1(ground, line, 10, 0.1, 0.5, 0.5))
			{
				std::cout << "��������ȡ3" << std::endl;
				lineClusters.push_back(line);
				it = jlClusters.erase(it);
				continue;
			}
			std::cout << "��������ȡ4" << std::endl;
		}
		++it;
	}
	std::cout << "other������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << std::endl;

	// ������ȡ�����ĳ��������3���������ཻС��2�ף�����Ϊ������
	std::vector<int> indices;
	std::vector<float> sqr_distances;
	for (auto it = jlClusters.begin(); it != jlClusters.end();)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr unknowclass(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
		extract.filter(*unknowclass);
		pcl::KdTreeFLANN<pcl::PointXYZRGB> unknowclass_kdtree;
		unknowclass_kdtree.setInputCloud(unknowclass);
		int insrt_size = 0;

		// ����ÿ�������ߣ��Ƿ��뵱ǰ�������С��һ��
		for (int j = 0; j < lineClusters.size(); ++j)
		{
			// ���������ÿһ����
			for (int k = 0; k < lineClusters[j].indices.indices.size(); ++k)
			{
				if (unknowclass_kdtree.radiusSearch(src_cloud->at(lineClusters[j].indices.indices[k]), 1, indices, sqr_distances))
				{
					insrt_size++;
					break;
				}
			}
		}
		if (insrt_size >= tower_intersectline_threshold)  // ��2�������ߺ����ཻ�ˣ���������ȷ��������������
		{
			towerClusters.push_back(pct::TowerInfo(*it));
			it = jlClusters.erase(it);
		}
		else
		{
			++it;
		}
	}
	// �ϲ�����
	std::cout << "�ϲ�����ǰother������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size() << std::endl;
	pct::MergeTower(src_cloud, ground_indices, jlClusters, lineClusters, towerClusters);
	std::cout << "�ϲ�����ǰother������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size() << std::endl;


	// ��������ߺ��������
	for (int i = 0; i < towerClusters.size(); i++)
	{
		pct::getMinMax3D(*src_cloud, towerClusters[i].indices, towerClusters[i].min, towerClusters[i].max);
		pcl::PointXYZRGB midpt;

		midpt.x = (towerClusters[i].min.x + towerClusters[i].max.x) / 2;
		midpt.y = (towerClusters[i].min.y + towerClusters[i].max.y) / 2;
		midpt.z = (towerClusters[i].min.z + towerClusters[i].max.z) / 2;
		towerClusters[i].cen = midpt;
	}
	std::cout << "�����������ĵ����" << std::endl;

	/*
	pcl::PointXYZRGB temppt;
	pct::TowerInfo tempcluster;
	for (int i = 0; i < (int)towerClusters.size() - 1; i++)
	{
	std::cout << "��������" << i << std::endl;
	pcl::PointXYZRGB minpt, maxpt;
	pct::getMinMax3D(*src_cloud, towerClusters[i].indices, minpt, maxpt);
	std::cout << "��ȡ��Χ��" << i << std::endl;
	for (int j = i + 1; j < (int)towerClusters.size() - 1; j++)
	{
	std::cout << "��ȡ��Χ��j" << i << " " << j << std::endl;
	pcl::PointXYZRGB minptj, maxptj;
	pct::getMinMax3D(*src_cloud, towerClusters[j].indices, minptj, maxptj);
	std::cout << "��ȡ��Χ��j" << minptj << std::endl;
	if (minpt.x < minptj.x)
	{
	std::cout << "�ƶ�����j" << i << " " << j << std::endl;
	tempcluster = towerClusters[i];
	towerClusters[i] = towerClusters[j];
	towerClusters[j] = tempcluster;
	}
	std::cout << "�ƶ�����j���" << i << " " << j << std::endl;
	}
	//towerClusters[i].tower_no = i + 1;
	std::cout << "�����������" << i << std::endl;
	}
	*/
	std::cout << "������ſ�ʼ" << std::endl;

	if (towerClusters.size())
		PositionCorrection(towerClusters);

	int vec_tower_series = 1;
	for (std::vector<pct::TowerInfo>::iterator vec_tower_it = towerClusters.begin(); vec_tower_it != towerClusters.end(); ++vec_tower_it)
	{
		if (vec_tower_it->tower_no == "")
		{
			vec_tower_it->tower_no = std::string("A") + QString::number(vec_tower_series).toLocal8Bit().data();
			vec_tower_series++;
		}
	}

	/*towerClusters[(int)towerClusters.size() - 1].tower_no = towerClusters.size();*/
	std::cout << "����������" << std::endl;
	for (int i = 0; i < lineClusters.size(); ++i)
	{
		pct::LineInfo &line = lineClusters[i];
		for (int j = 0; j < towerClusters.size(); ++j)
		{
			if (pct::Distance2d(line.sta, towerClusters[j].cen) < 25)
			{
				line.begin_tower_no = towerClusters[j].tower_no;
			}
			else if (pct::Distance2d(line.end, towerClusters[j].cen) < 25)
			{
				line.end_tower_no = towerClusters[j].tower_no;
			}
		}
	}
	std::cout << "�����߱�����" << std::endl;

	//  ��ȡֲ��
	pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
	if (ground->size())
	{
		ground_kdtree.setInputCloud(ground);
		indices.clear();
		sqr_distances.clear();
		for (auto it = jlClusters.begin(); it != jlClusters.end();)
		{
			pct::VegetInfo veg = pct::vegetInfoFactory(src_cloud, *it);
			// ��͵�����ظ߶�<10���Ҵ���30���㣬�п�����ֲ�
			if (LikeVeget(ground, veg, 0.3, 0.5, 0.5))
			{
				vegetClusters.push_back(veg);
				it = jlClusters.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

	std::cout << "other������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size()
		<< "ֲ��������" << vegetClusters.size() << "\nһ��" << jlClusters.size() + lineClusters.size() + towerClusters.size() + vegetClusters.size() << std::endl;


	//// ȫ���㶼Ĭ�Ϻ�ɫ
	//for (auto it = src_cloud->begin(); it != src_cloud->end(); ++it)
	//{
	//    it->r = 0;
	//    it->g = 0;
	//    it->b = 0;
	//}

	// �������ɫ
	unsigned int color = setting.cls_intcolor(ground_str);
	unsigned char r = *(((unsigned char *)&color) + 2);
	unsigned char g = *(((unsigned char *)&color) + 1);
	unsigned char b = *(((unsigned char *)&color) + 0);
	pcl::PointXYZRGB *tmppt;
	for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
	{
		tmppt = &src_cloud->at(*it);
		tmppt->r = r;
		tmppt->g = g;
		tmppt->b = b;
	}

	// ��������ɫ
	color = setting.cls_intcolor(power_line_str);
	r = *(((unsigned char *)&color) + 2);
	g = *(((unsigned char *)&color) + 1);
	b = *(((unsigned char *)&color) + 0);

	for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
	{
		for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
		{
			tmppt = &src_cloud->at(*itt);
			tmppt->r = r;
			tmppt->g = g;
			tmppt->b = b;
		}
	}
	// ������ɫ
	color = setting.cls_intcolor(tower_str);
	r = *(((unsigned char *)&color) + 2);
	g = *(((unsigned char *)&color) + 1);
	b = *(((unsigned char *)&color) + 0);
	for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
	{
		for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
		{
			tmppt = &src_cloud->at(*itt);
			tmppt->r = r;
			tmppt->g = g;
			tmppt->b = b;
		}
		std::cout << "��⵽�������࣬����Ϊ��" << it->indices.indices.size() << std::endl;
	}

	// ֲ������ɫ
	color = setting.cls_intcolor(veget_str);
	r = *(((unsigned char *)&color) + 2);
	g = *(((unsigned char *)&color) + 1);
	b = *(((unsigned char *)&color) + 0);
	for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
	{
		for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
		{
			tmppt = &src_cloud->at(*itt);
			tmppt->r = r;
			tmppt->g = g;
			tmppt->b = b;
		}
	}

	// ��������ɫ
	color = setting.cls_intcolor(others_str);
	r = *(((unsigned char *)&color) + 2);
	g = *(((unsigned char *)&color) + 1);
	b = *(((unsigned char *)&color) + 0);
	for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
	{
		for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
		{
			tmppt = &src_cloud->at(*itt);
			tmppt->r = r;
			tmppt->g = g;
			tmppt->b = b;
		}
	}

	pct::io::save_las(src_cloud, outputfile);




	// ��������� .las
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr lines_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
		extract.filter(*temp_cloud);
		//pcl::PointXYZRGB *temp_pt;
		//unsigned char r = rand() % 256;
		//unsigned char g = rand() % 256;
		//unsigned char b = rand() % 256;
		//
		//for (int j = 0; j < temp_cloud->size(); ++j)
		//{
		//    temp_pt = &temp_cloud->at(j);
		//    temp_pt->r = r;
		//    temp_pt->g = g;
		//    temp_pt->b = b;
		//}

		*lines_cloud += *temp_cloud;
	}
	pct::io::save_las(lines_cloud, setting.outputdir + "\\lines.las");
	lines_cloud.reset();
	pct::ConvGeopnts(setting.outputdir + "\\lines.las");

	// �������� .las
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tower_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
		extract.filter(*temp_cloud);
		*tower_cloud += *temp_cloud;
	}
	if (tower_cloud->size())
	{
		pct::io::save_las(tower_cloud, setting.outputdir + "\\towers.las");
		tower_cloud.reset();
		pct::ConvGeopnts(setting.outputdir + "\\towers.las");
	}



	// ����vegets .las
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr vegets_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
		extract.filter(*temp_cloud);
		*vegets_cloud += *temp_cloud;
	}
	if (vegets_cloud->size())
	{
		pct::io::save_las(vegets_cloud, setting.outputdir + "\\vegets.las");
		vegets_cloud.reset();
		pct::ConvGeopnts(setting.outputdir + "\\vegets.las");
	}

	// ����others .las
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr others_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
		extract.filter(*temp_cloud);
		*others_cloud += *temp_cloud;
	}
	if (others_cloud->size())
	{
		pct::io::save_las(others_cloud, setting.outputdir + "\\others.las");
		others_cloud.reset();
		pct::ConvGeopnts(setting.outputdir + "\\others.las");
	}


	// ����ground .las
	ground->clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(boost::make_shared<std::vector<int>>(ground_indices->indices));
	extract.filter(*ground_cloud);
	if (ground_cloud->size())
	{
		pct::io::save_las(ground_cloud, setting.outputdir + "\\ground.las");
		ground_cloud.reset();
		pct::ConvGeopnts(setting.outputdir + "\\ground.las");
	}


	std::cout << "ExtractLinesAndTower end��end" << std::endl;
}

