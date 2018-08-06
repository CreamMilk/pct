#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <chrono>   
#include <Scene_points_with_normal_item.h> //��
#include <Item_classification_base.h>
#include <Point_set_item_classification.h>
#include <QFileInfo>
#include <QMessageBox>

#include "commonfunctions.h"
#include "setting.hpp"
#include "pctio.h"


//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // ���س���
bool ParserCmdline(int argc, char *argv[]);
bool train();
bool classif();

int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
    const pct::Setting & setting = pct::Setting::ins();
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // ���ѵ���ļ�Ϊ��,����Ҫ������ѵ��
    if (_access((setting.classdir + "\\config.xml").c_str(), 0) == -1 || setting.retrain)
    {
        train();
    }

    // ����
    classif();

    return EXIT_SUCCESS;
}

bool ParserCmdline(int argc, char *argv[])
{
    pct::Setting& setting = pct::Setting::ins();
    // ����������
    boost::program_options::options_description opts("pointcloud tool options");
    boost::program_options::variables_map vm;

    opts.add_options()
        ("inputfile", boost::program_options::value<std::string>(), "����·����*.las *.xyz *.ply��")
        ("classdir", boost::program_options::value<std::string>(), "�����ļ�Ŀ¼ <dir>")
        ("outputdir", boost::program_options::value<std::string>(), "������Ŀ¼ [dir]")
        ("retrain", boost::program_options::value<bool>(), "����ѵ������ [bool]")
        ("method", boost::program_options::value<int>(), "���෽�� [int] 0-��ͨ 1-ƽ�� 2-�ǳ�ƽ��")
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
        setsettingitem(inputfile, std::string, true, ""); 
        boost::filesystem::path path_file(inputfile);
        boost::filesystem::path path_dir(path_file.parent_path().string());
       
        if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
        {
            std::cout << "ѡ��inputfile�ļ�·������"  << std::endl; 
            std::cout <<  inputfile << std::endl;
            return false;
        }

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

        setsettingitem(classdir, std::string, true, "");
        if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //����ļ��в����ڣ����߲����ļ���
        {
            std::cout << "ѡ��classdir�ļ���·������" << std::endl;
            std::cout << classdir << std::endl;
            return false;
        }
        setsettingitem(retrain, bool, false, false);

        setsettingitem(method, int, false, 0);
    }

    return true;
}

bool train()
{
    std::cout << "����ѵ��..." << std::endl;
    const pct::Setting & setting = pct::Setting::ins();
    int nb_scales = setting.value<int>("nb_scales");
    int nb_trials = setting.value<int>("nb_trials");
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
                if (boost::filesystem::is_regular_file(*fileiter) && boost::algorithm::to_lower_copy(boost::filesystem::extension(*fileiter))== ".las")
                {
                    std::string laspath = fileiter->path().string(); // �õ��ļ�·��
                    std::string lasname = fileiter->path().stem().string(); // �õ��ļ���
                    std::cout << "�ļ���" << lasname << "..." << std::endl;
                    // ���ص��� ,������Ƿ�����ɫ��Ϣ
                    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(laspath));
                    if (scene_item && scene_item->point_set()->check_colors())
                    {
                        auto start = std::chrono::system_clock::now();
                        Point_set* points = scene_item->point_set();
                        // ��������
                        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
                        classif->compute_features(nb_scales);

                        // ���label
                        classif->add_new_label(classname.c_str());
                        classif->add_new_label("unselect");

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

    std::string labelname_traverse;
    std::string config_xml = setting.classdir + "\\config.xml";
    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(setting.inputfile));
    if (scene_item)
    {
        // ��������
        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
        classif->compute_features(nb_scales);

        // ���label
        std::string labelname;
        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(config_xml, pt);
        BOOST_AUTO(labels, pt.get_child("classification.labels"));
        for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
        {
            labelname_traverse = label->second.get<std::string>("name");
            classif->add_new_label(labelname_traverse.c_str(), setting.cls_color(labelname_traverse));
        }

        // ����
        classif->load_config(config_xml.c_str(), 0);
        classif->run(method, 0, 16, 0.5);
        pct::io::lassave(scene_item.get(), setting.outputdir + "\\out.las");
    }
    return true;
}

