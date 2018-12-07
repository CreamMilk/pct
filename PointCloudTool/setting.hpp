#ifndef SETTING_HPP
#define SETTING_HPP

 #include <boost/property_tree/ptree.hpp>
 #include <boost/property_tree/json_parser.hpp>
 #include <boost/foreach.hpp>
#include <QStringList>
#include <QMessageBox>
#include <tuple>
#include <map>
#include <boost/typeof/typeof.hpp>

namespace pct
{
    struct Setting
    {
        static Setting& ins()
        {
            static Setting s;
            return s;
        }

    private:
        Setting()
            :gridsize(0),
            reclassif(false),
            overrideExcel(false),
            stampcorrectcell(true)
        {
            // 获得程序目录
            char *tmpstr;
            _get_pgmptr(&tmpstr);
            char drive[_MAX_DRIVE];
            char dir[_MAX_DIR];
            char fname[_MAX_FNAME];
            char ext[_MAX_EXT];
            _splitpath(tmpstr, drive, dir, fname, ext);
            appdir = std::string(drive) + dir;

            // 读取配置文件
            std::ifstream is(appdir + "\\pctconfig.json", std::ios::in);
            if (is.is_open())
            {
                try {
                    read_json(is, pt);          //parse json
                    is.close();
                }
                catch (...) {
                    return;
                }
                
                is.close();
            }
        }

       

    public:
        template<class Type>
        Type value(const std::string &key) const
        {
            return pt.get_optional<Type>(key).value();
        }

        std::string cls_strcolor(const std::string &cls) const
        {
            QString color_str = pt.get_child("classif_color").get_optional<std::string>(cls).value().c_str();
            return color_str.toLocal8Bit().data();
        }

        unsigned int cls_intcolor(const std::string &cls) const
        {
            QString color_str = pt.get_child("classif_color").get_optional<std::string>(cls).value().c_str();
            color_str.remove(' ');
            QStringList l = color_str.split(',');

            if (l.size() != 3)
            {
                return rand();
            }
            else
            {
                unsigned int colorint = 0;
                *(((unsigned char *)&colorint) + 2) = l[0].toUInt();
                *(((unsigned char *)&colorint) + 1) = l[1].toUInt();
                *(((unsigned char *)&colorint) + 0) = l[2].toUInt();
                return colorint;
            }
        }

        std::map<std::string, std::tuple<float, float>> get_distances() const
        {
            // 水平距离， 垂直距离
            std::map<std::string, std::tuple<float, float>> distances;

            BOOST_AUTO(dangerdistance, pt.get_child("dangerdistance"));
           
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &leixing, dangerdistance)
            {
                QString key(leixing.first.c_str());
                QString val(leixing.second.get_value_optional<std::string>().value().c_str());

                val.remove(" ");
                QStringList split_val = val.split(',');
                distances.insert(std::make_pair(leixing.first, std::tuple<float, float>(split_val[0].toFloat(), split_val[1].toFloat())));
            }
            return distances;
        }

        // 配置文件
        boost::property_tree::ptree pt;                       //define property_tree object
        // 程序路径之类
        std::string appdir;

        // 命令行参数
        std::string cmdtype;
        std::string inputfile; //点云路径｛*.las *.xyz *.ply｝
        std::string outputdir; //样本文件目录 <dir>
        std::string classdir; //输出结果目录 [dir]
        int method; //分类处理方法 [int]
        float gridsize;
        bool reclassif;
        std::string exceldir; // 纠偏塔数据目录 [std::string]
        bool overrideExcel;  // 是否覆盖源excel [bool]
        bool stampcorrectcell;  // 是否标记修改cell [bool]
		QString tower_excle;
		QString line_excel;
    };
  



}

#endif