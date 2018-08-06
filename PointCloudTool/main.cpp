#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <chrono>   
#include <Scene_points_with_normal_item.h> //点
#include <Item_classification_base.h>
#include <Point_set_item_classification.h>
#include <QFileInfo>
#include <QMessageBox>

#include "commonfunctions.h"
#include "setting.hpp"
#include "pctio.h"


//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // 隐藏程序
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

    // 如果训练文件为空,或者要求重新训练
    if (_access((setting.classdir + "\\config.xml").c_str(), 0) == -1 || setting.retrain)
    {
        train();
    }

    // 分类
    classif();

    return EXIT_SUCCESS;
}

bool ParserCmdline(int argc, char *argv[])
{
    pct::Setting& setting = pct::Setting::ins();
    // 解析命令行
    boost::program_options::options_description opts("pointcloud tool options");
    boost::program_options::variables_map vm;

    opts.add_options()
        ("inputfile", boost::program_options::value<std::string>(), "点云路径｛*.las *.xyz *.ply｝")
        ("classdir", boost::program_options::value<std::string>(), "样本文件目录 <dir>")
        ("outputdir", boost::program_options::value<std::string>(), "输出结果目录 [dir]")
        ("retrain", boost::program_options::value<bool>(), "重新训练样本 [bool]")
        ("method", boost::program_options::value<int>(), "分类方法 [int] 0-普通 1-平滑 2-非常平滑")
        ("help", "帮助");

    try{
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    }
    catch (...){
        std::cout << "输入的参数中存在未定义的选项！\n";
        return false;
    }

    // 检查并赋值
    if (vm.count("help")){//若参数中有help选项
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
            std::cout << "选项inputfile文件路径错误！"  << std::endl; 
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
            std::cout << "选项outputdir目录错误！" << std::endl;
            std::cout << outputdir << std::endl;
            return false;
        }

        setsettingitem(classdir, std::string, true, "");
        if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //如果文件夹不存在，或者不是文件夹
        {
            std::cout << "选项classdir文件夹路径错误！" << std::endl;
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
    std::cout << "重新训练..." << std::endl;
    const pct::Setting & setting = pct::Setting::ins();
    int nb_scales = setting.value<int>("nb_scales");
    int nb_trials = setting.value<int>("nb_trials");
    // 遍历样本文件夹
    boost::filesystem::path yangbendir(setting.classdir);
    std::cout << "setting.classdir：" << setting.classdir << std::endl;
    boost::filesystem::directory_iterator iter_dirend;

    std::vector<std::string> labels;
    std::vector<std::string> labelxmls;
    for (boost::filesystem::directory_iterator yangbendiriter(yangbendir); yangbendiriter != iter_dirend; ++yangbendiriter)
    {
        // 遍历类型下边的所有样本文件
        if (boost::filesystem::is_directory(*yangbendiriter))
        {
            std::vector<std::string> lasxmls;
            std::string classdir = yangbendiriter->path().string(); // 得到类文件夹路径
            std::string classname = yangbendiriter->path().stem().string();  // 得到类名
            std::string labelxml = classdir + "\\config.xml";
            std::cout << "开始训练" << classname << "..." << std::endl;
            for (boost::filesystem::directory_iterator fileiter(*yangbendiriter); fileiter != iter_dirend; ++fileiter)
            {
                // 如果文件有效，而且后缀名为las，则是我们要训练的东西
                if (boost::filesystem::is_regular_file(*fileiter) && boost::algorithm::to_lower_copy(boost::filesystem::extension(*fileiter))== ".las")
                {
                    std::string laspath = fileiter->path().string(); // 得到文件路径
                    std::string lasname = fileiter->path().stem().string(); // 得到文件名
                    std::cout << "文件：" << lasname << "..." << std::endl;
                    // 加载点云 ,并检查是否含有颜色信息
                    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(laspath));
                    if (scene_item && scene_item->point_set()->check_colors())
                    {
                        auto start = std::chrono::system_clock::now();
                        Point_set* points = scene_item->point_set();
                        // 计算特征
                        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
                        classif->compute_features(nb_scales);

                        // 添加label
                        classif->add_new_label(classname.c_str());
                        classif->add_new_label("unselect");

                       // 选择红色的
                        classif->change_color(0);
                        std::vector<Point_set::Index> unselected, selected;
                        for (Point_set::Index idx : *points)
                        {
                            if (points->red(idx) == 1. && scene_item->point_set()->green(idx) == 0. && scene_item->point_set()->blue(idx) == 0.)
                                selected.push_back(idx);
                            else
                                unselected.push_back(idx);
                        }

                        // 选择,加入label
                        scene_item->selectPoints(selected, unselected);
                        std::cout << "selected red points： " << points->number_of_removed_points() << std::endl;
                        classif->add_selection_to_training_set(0);  

                        // 非选择,加入unselectlabel
                        scene_item->selectPoints(unselected, selected);
                        //scene_item->invertSelection();  // 切记add_selection_to_training_set会删除选择集，所以这个函数在这里不能用
                        std::cout << "unselect points： " << points->number_of_removed_points() << std::endl;
                        classif->add_selection_to_training_set(1);

                        // 训练
                        std::cout << "训练" << classname << "-" << lasname << std::endl;
                        classif->train(0, nb_trials, 0, 0);
                        std::string lasxml_path = classdir + "\\" + lasname + ".xml";
                        classif->save_config(lasxml_path.c_str(), 0);
                        lasxmls.push_back(lasxml_path);

                        std::cout << "训练" << classname << "完成。一共" << points->size() << "个点,选择" << selected.size() << "个。\n用时"
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
    std::cout << "开始分类..." << std::endl;
    const pct::Setting & setting = pct::Setting::ins();
    int nb_scales = setting.value<int>("nb_scales");
    int method = setting.method;

    std::string labelname_traverse;
    std::string config_xml = setting.classdir + "\\config.xml";
    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(setting.inputfile));
    if (scene_item)
    {
        // 计算特征
        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
        classif->compute_features(nb_scales);

        // 添加label
        std::string labelname;
        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(config_xml, pt);
        BOOST_AUTO(labels, pt.get_child("classification.labels"));
        for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
        {
            labelname_traverse = label->second.get<std::string>("name");
            classif->add_new_label(labelname_traverse.c_str(), setting.cls_color(labelname_traverse));
        }

        // 分类
        classif->load_config(config_xml.c_str(), 0);
        classif->run(method, 0, 16, 0.5);
        pct::io::lassave(scene_item.get(), setting.outputdir + "\\out.las");
    }
    return true;
}

