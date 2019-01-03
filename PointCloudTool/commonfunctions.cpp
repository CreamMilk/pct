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

	uchar staC[3] = { 0, 255, 255 };
	uchar endC[3] = { 255, 0, 0 };
	short subR = endC[0] - staC[0];
	short subG = endC[1] - staC[1];
	short subB = endC[2] - staC[2];


	double heig = max.z - min.z;
	if (heig == 0)
		return;

	std::cout << "ScreenshotHeightColor heig = " << heig << "\t cloud->size() = " << cloud->size() << std::endl;
	for (int i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZRGB &pt = cloud->at(i);
		double l = (pt.z - min.z) / heig;
		pt.r = (subR)* (l)+staC[0];
		pt.b = (subG)* (l)+staC[1];
		pt.g = (subB)* (l)+staC[2];
	}


	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("HeightColor"));

	view->setBackgroundColor(0,0,0);
	view->addPointCloud<pcl::PointXYZRGB>(cloud, "HeightColorCloud");      // no need to add the handler, we use a random handler by default
	view->resetCamera();

	pcl::visualization::Camera camera;
	view->getCameraParameters(camera);
	float cameralen = (vec(camera.focal[0], camera.focal[1], camera.focal[2]) - vec(camera.pos[0], camera.pos[1], camera.pos[2])).Length();

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
