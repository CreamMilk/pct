#include <QtCore/QCoreApplication>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <iostream>
#include <iosfwd>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/uniform_sampling.h>   //均匀采样
#include <pcl/filters/statistical_outlier_removal.h>

void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
void Save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
pcl::PointXYZRGB GetMidPt(pcl::PointXYZRGB minpt, pcl::PointXYZRGB maxpt);
void simple(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float gridsize);
void OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

int main(int argc, char **argv)
{
    QCoreApplication a(argc, argv);
   
    if (strcmp(argv[1], "help") == 0)
        std::cout << "参数1：合并目录" << std::endl;

    boost::filesystem::path mergedir(argv[1]);
    if ((argc!=2) || (!boost::filesystem::is_directory(mergedir)))
        return 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr totol_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int file_step = 0;
    int xoff = 0;  // 每隔一千五百米放一个点云
    int yoff = 0;
    boost::filesystem::directory_iterator iter_dirend;
    for (boost::filesystem::directory_iterator fileiter(mergedir); fileiter != iter_dirend; ++fileiter)
    {
        // 如果文件有效，而且后缀名为las，则是我们要训练的东西
        if (boost::filesystem::is_regular_file(*fileiter) && boost::algorithm::to_lower_copy(boost::filesystem::extension(*fileiter)) == ".las")
        {
            std::string laspath = fileiter->path().string(); // 得到文件路径
            std::string lasname = fileiter->path().stem().string(); // 得到文件名
            if (lasname == "mergeCloud")
                continue;

            std::cout << lasname << std::endl;
            if (file_step % 5 == 0)
            {
                yoff -= 1000;
                xoff = 0;
            }

            // 加载点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr las_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            Load_las(las_cloud, laspath);
            simple(las_cloud, 0.3);
            OutlierRemoval(las_cloud);

            // 计算中心点
            pcl::PointXYZRGB minpt, maxpt, midpt;
            pcl::getMinMax3D(*las_cloud, minpt, maxpt);
            midpt = GetMidPt(minpt, maxpt);
            
            for (auto it = las_cloud->begin(); it != las_cloud->end(); ++it)
            {
                it->x = it->x - midpt.x + xoff;
                it->y = it->y - midpt.y + yoff;
                it->z = it->z - midpt.z;
            }
            std::cout << las_cloud->at(0) << std::endl;
            *totol_cloud += *las_cloud;

            xoff += 1000;
            file_step++;
        }
    }

    Save_las(totol_cloud, std::string(argv[1]) + "\\mergeCloud.las");
    return /*a.exec()*/0;
}

pcl::PointXYZRGB GetMidPt(pcl::PointXYZRGB minpt, pcl::PointXYZRGB maxpt)
{
    pcl::PointXYZRGB midpt;
    midpt.x = (minpt.x + maxpt.x)/2;
    midpt.y = (minpt.y + maxpt.y)/2;
    midpt.z = (minpt.z + maxpt.z)/2;
    return midpt;
}

void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
{
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(path.c_str());
    if (!lasreadopener.active())
    {
        return;
    }
    LASreader* lasreader = lasreadopener.open();
    cloud->resize(lasreader->header.number_of_point_records);
    size_t step = 0;
    pcl::PointXYZRGB *pt;
    while (lasreader->read_point())
    {
        pt = &cloud->at(step);
        pt->x = lasreader->point.get_x();
        pt->y = lasreader->point.get_y();
        pt->z = lasreader->point.get_z();

        //std::cout << "int" << lasreader->point.get_R() / (256) << std::endl;
        //std::cout << lasreader->point.get_R() / (double)(256) << std::endl;
        pt->r = lasreader->point.get_R() / (256);
        pt->g = lasreader->point.get_G() / (256);
        pt->b = lasreader->point.get_B() / (256);
        if (pt->r == 0 && pt->g == 0 && pt->b == 0)  // 因为cgal需要两种以上不为黑的的颜色，才认为las含有颜色信息。  所以不要默认色设置成黑色。
        {
            pt->r = 1;
            pt->g = 1;
            pt->b = 1;
        }
        ++step;
    }
    lasreader->close();
    delete lasreader;

}

void Save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
{
    LASheader lasheader;
    lasheader.x_offset = 0;
    lasheader.y_offset = 0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 3;	//1是xyzrgb		,颜色设置不上，未找到原因暂时屏蔽掉
    lasheader.point_data_record_length = 34;

    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(path.c_str());
    if (!laswriteopener.active())
    {
        return;
    }

    // init point 

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    // open laswriter

    LASwriter* laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0)
    {
        fprintf(stderr, "ERROR: could not open laswriter\n");
        return;
    }
    for (auto it = cloud->begin(); it != cloud->end(); ++it)
    {
        laspoint.set_x(it->x);
        laspoint.set_y(it->y);
        laspoint.set_z(it->z);
        laspoint.set_R(it->r * 256);
        laspoint.set_G(it->g * 256);
        laspoint.set_B(it->b * 256);
        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);
    }

    // update the header
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
}

void simple(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float gridsize)
{
    //均匀采样点云并提取关键点      体素下采样，重心代替
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);  //输入点云
    uniform_sampling.setRadiusSearch(gridsize);   //设置半径 
    uniform_sampling.filter(*cloud);   //滤波
}

void OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    int liqunK = 10;
    double avgDistance = 10;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
    sor.setInputCloud(cloud);                           //设置待滤波的点云
    sor.setMeanK(liqunK);                               //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(avgDistance);                      //设置判断是否为离群点的阀值

    sor.filter(*cloud);
}