#pragma once
#include <QFileInfo>
#include "Scene_points_with_normal_item.h"

namespace pct
{
    namespace io
    {
        // º”‘ÿlas
        Scene_points_with_normal_item* lasload(const std::string& file_path);

        // ±£¥Êlas
        bool lassave(const Scene_points_with_normal_item* item, const std::string& file_path);
    }

}

