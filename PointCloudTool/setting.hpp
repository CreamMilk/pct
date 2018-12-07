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
            // ��ó���Ŀ¼
            char *tmpstr;
            _get_pgmptr(&tmpstr);
            char drive[_MAX_DRIVE];
            char dir[_MAX_DIR];
            char fname[_MAX_FNAME];
            char ext[_MAX_EXT];
            _splitpath(tmpstr, drive, dir, fname, ext);
            appdir = std::string(drive) + dir;

            // ��ȡ�����ļ�
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
            // ˮƽ���룬 ��ֱ����
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

        // �����ļ�
        boost::property_tree::ptree pt;                       //define property_tree object
        // ����·��֮��
        std::string appdir;

        // �����в���
        std::string cmdtype;
        std::string inputfile; //����·����*.las *.xyz *.ply��
        std::string outputdir; //�����ļ�Ŀ¼ <dir>
        std::string classdir; //������Ŀ¼ [dir]
        int method; //���ദ���� [int]
        float gridsize;
        bool reclassif;
        std::string exceldir; // ��ƫ������Ŀ¼ [std::string]
        bool overrideExcel;  // �Ƿ񸲸�Դexcel [bool]
        bool stampcorrectcell;  // �Ƿ����޸�cell [bool]
		QString tower_excle;
		QString line_excel;
    };
  



}

#endif