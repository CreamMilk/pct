#include "pctio.h"

#include <fstream>
#include <iostream>
#include <iosfwd>
#include <QFileInfo>

#include <lasreader.hpp>
#include <laswriter.hpp>


Scene_points_with_normal_item* pct::io::lasload(const std::string& file_path)
{
    std::ifstream in(file_path, std::ios_base::binary);

    if (!in)
        std::cerr << "Error!\n";
    
    Scene_points_with_normal_item* item;
    item = new Scene_points_with_normal_item();
    if (!item->read_las_point_set(in))
    {
        delete item;
        return nullptr;
    }

    //item->setName(fileinfo.completeBaseName());
    return item;
}

bool pct::io::lassave(const Scene_points_with_normal_item* item, const std::string& file_path)
{
    // This plugin supports point sets
    const Scene_points_with_normal_item* point_set_item =
        qobject_cast<const Scene_points_with_normal_item*>(item);
    if (!point_set_item)
        return false;

    std::ofstream out(file_path, std::ios_base::binary);
    return point_set_item->write_las_point_set(out);
}
 
LASheader pct::io::Get_las_header(std::string path)
{
    LASreadOpener lasreadopener;
    LASheader header;
    lasreadopener.set_file_name(path.c_str());
    if (!lasreadopener.active())
    {
        return header;
    }
    LASreader* lasreader = lasreadopener.open();
    header = lasreader->header;
    lasreader->close();
    //delete lasreader;
    return header;
}

void pct::io::Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
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

void pct::io::save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
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
