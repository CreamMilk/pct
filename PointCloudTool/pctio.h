#pragma once
#include <QFileInfo>
#include "Scene_points_with_normal_item.h"

namespace pct
{
    namespace io
    {
        // ����las
        Scene_points_with_normal_item* lasload(const std::string& file_path);

        // ����las
        bool lassave(const Scene_points_with_normal_item* item, const std::string& file_path);
    }

}

