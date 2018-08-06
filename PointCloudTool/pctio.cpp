#include "pctio.h"

#include <fstream>
#include <iostream>
#include <iosfwd>
#include <QFileInfo>

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
 
