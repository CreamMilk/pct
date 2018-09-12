#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <boost/locale.hpp>
#include <chrono>   
#include <Scene_points_with_normal_item.h> //点
#include <Item_classification_base.h>
#include <Point_set_item_classification.h>
#include <QFileInfo>
#include <QMessageBox>

#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "commonfunctions.h"
#include "setting.hpp"
#include "pctio.h"
#include "mydef.h"
#include "DangerousDistanceCheck.h"

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <vtkOutputWindow.h>
#include <QVTKOpenGLWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <QSurfaceFormat>

//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // 隐藏程序
bool ParserCmdline(int argc, char *argv[]);
bool train();
bool classif();
void correct();
void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices>& jlClusters, std::vector <pcl::PointIndices>& lineClusters, std::vector <pcl::PointIndices>& towerClusters);
bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,  pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices>& otheCluster, std::vector <pcl::PointIndices>& lineClusters, std::vector <pcl::PointIndices>& towerClusters, double dangerousDistance);

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8); //1     

    srand((unsigned)time(NULL));
    const pct::Setting & setting = pct::Setting::ins();
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }


    if (setting.cmdtype == "train")
    {
        train();
    }
    else if (setting.cmdtype == "classif")
    {
        classif();
        correct();
    }
    else if (setting.cmdtype == "distancecheck")
    {
        classif();
        const pct::Setting & setting = pct::Setting::ins();
        std::string inputfile = setting.outputdir + "\\out.las";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pct::io::Load_las(src_cloud, inputfile);
        std::cout << "离群点过滤" << std::endl;
        pct::OutlierRemoval(src_cloud);

        // 因为cgal分类结果是整体的，并不能把每个个体提取出来
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pcl::PointIndices> lineClusters;
        std::vector <pcl::PointIndices> towerClusters;
       
        correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters);
        checkLinesDistanceDangerous(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, 5);
    }
    else
    {
        return EXIT_FAILURE;
    }
    

    return EXIT_SUCCESS;
}

void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& otheCluster,
    std::vector <pcl::PointIndices>& lineClusters,
    std::vector <pcl::PointIndices>& towerClusters,
    double dangerousDistance)
{
    DangerousDistanceCheck ddc;
    ddc.setData(src_cloud 
        , ground_indices
        , otheCluster
        , lineClusters
        , towerClusters
        , dangerousDistance);
    ddc.TooNearCheck();
    ddc.showNearCheck();
    std::cout << "showNearCheck endl." << std::endl;
}

bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    setsettingitem(classdir, std::string, true, "");
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //如果文件夹不存在，或者不是文件夹
    {
        std::cout << "选项classdir文件夹路径错误！" << std::endl;
        std::cout << classdir << std::endl;
        return false;
    }
    setsettingitem(gridsize, float, false, 0.3);

    return true;
}

bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    setsettingitem(inputfile, std::string, true, "");
    boost::filesystem::path path_file(inputfile);
    boost::filesystem::path path_dir(path_file.parent_path().string());

    if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
    {
        std::cout << "选项inputfile文件路径错误！" << std::endl;
        std::cout << inputfile << std::endl;
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
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //如果文件夹不存在，或者不是文件夹
    {
        std::cout << "样本文件夹中没有训练文件config.xml，请先训练样本！" << std::endl;
        std::cout << classdir << std::endl;
        return false;
    }
    setsettingitem(method, int, false, 0);

    setsettingitem(gridsize, float, false, 0.3);

    return true;
}

bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    if (!ReadyClassifOpts(setting, vm))
    {
        return false;
    }

    return true;
}

bool ParserCmdline(int argc, char *argv[])
{
    std::string exe_name = pct::GetExeName();
    pct::Setting& setting = pct::Setting::ins();
    // 解析命令行
    boost::program_options::variables_map vm;
     boost::program_options::options_description opts("pointcloud tool options"
         "\n示例："
         "\n（1）" + exe_name + " --cmdtype train --classdir classdir --gridsize 0.3"
         "\n（2）" + exe_name + " --cmdtype classif --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n（3）" + exe_name + " --cmdtype distancecheck --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n参数");

    opts.add_options()
        ("cmdtype", boost::program_options::value<std::string>(), "命令类型｛train classif distancecheck｝")
        ("inputfile", boost::program_options::value<std::string>(), "点云路径｛*.las *.xyz *.ply｝")
        ("classdir", boost::program_options::value<std::string>(), "样本文件目录 <dir>")
        ("outputdir", boost::program_options::value<std::string>(), "输出结果目录 [dir]")
        ("method", boost::program_options::value<int>(), "分类方法 [int] 0-普通 1-平滑 2-非常平滑")
        ("gridsize", boost::program_options::value<float>(), "叶节点尺寸 [float]")
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
        setsettingitem(cmdtype, std::string, true, "");
        if (cmdtype == "train")
        {
            if (!ReadyTrainOpts(setting, vm))
                return false;
        }
        else if (cmdtype == "classif")
        {
            if (!ReadyClassifOpts(setting, vm))
                return false;
        }
        else if (cmdtype == "distancecheck")
        {
            if (!ReadyDistancecheckOpts(setting, vm))
                return false;
        }
        else
        {
            std::cout << "参数cmdtype输入错误" << std::endl;
            return false;
        }
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
                    
                    // 抽稀  
                    std::ostringstream tempfile;
                    tempfile << getpid() << "temp2018.las";
                    pct::simple(laspath, tempfile.str(), setting.gridsize);
                    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));
                    boost::filesystem::remove(boost::filesystem::path(tempfile.str()));

                    std::cout << "文件：" << lasname << "..." << std::endl;
                    if (scene_item && scene_item->point_set()->check_colors())
                    {
                        auto start = std::chrono::system_clock::now();
                        Point_set* points = scene_item->point_set();
                        // 计算特征
                        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
                        classif->compute_features(nb_scales);

                        // 添加label
                        classif->add_new_label(classname.c_str());
                        classif->add_new_label(unselect_str);

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


    // 抽稀  
    std::ostringstream tempfile;
    tempfile << getpid() << "temp2018.las";
    pct::simple(setting.inputfile, tempfile.str(), setting.gridsize);
    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));
    boost::filesystem::remove(boost::filesystem::path(tempfile.str()));

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
            classif->add_new_label(labelname_traverse.c_str(), setting.cls_intcolor(labelname_traverse));
        }

        // 分类
        classif->load_config(config_xml.c_str(), 0);
        classif->run(method, 0, 16, 0.5);
        pct::io::lassave(scene_item.get(), setting.outputdir + "\\out.las");
    }
    return true;
}

void correct()
{
    const pct::Setting & setting = pct::Setting::ins();
    std::string inputfile = setting.outputdir + "\\out.las";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pct::io::Load_las(src_cloud, inputfile);
    std::cout << "离群点过滤" << std::endl;
    //pct::OutlierRemoval(src_cloud);

    // 因为cgal分类结果是整体的，并不能把每个个体提取出来
    std::vector <pcl::PointIndices> otheCluster;
    std::vector <pcl::PointIndices> lineClusters;
    std::vector <pcl::PointIndices> towerClusters;
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters);
}

void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& jlClusters,
    std::vector <pcl::PointIndices>& lineClusters,
    std::vector <pcl::PointIndices>& towerClusters)
{
    const pct::Setting & setting = pct::Setting::ins();
   
    std::string outputfile = setting.outputdir + "\\out.las";
    const int tower_intersectline_threshold = 3;

    std::cout << "correct begin" << std::endl;
    pct::io::save_las(src_cloud, setting.outputdir + "\\cgalclassif.las");


    // 提取地面点索引
    pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
    pct::ExtractGround(src_cloud, cloud_indices, ground_indices);
    std::cout << "非地面点：" << cloud_indices->indices.size()
        << "地面点：" << ground_indices->indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(ground_indices);
    extract.filter(*ground);


    // 再对剩余的点颜色聚类
    pct::colorClusters(src_cloud, *cloud_indices, jlClusters);
    std::cout << "聚类数量：" << jlClusters.size() << std::endl;

    // 电力线提取
    // 聚类结果是否含有70%以上的电力线点，如果是，就说明是电力线点
    for (auto it = jlClusters.begin(); it != jlClusters.end(); )
    {
        if (pct::pointsCountsForColor(src_cloud, *it, setting.cls_intcolor(power_line_str)) > it->indices.size()*0.25)  // power_line":"255, 255, 0
        {
            pct::LineInfo line = pct::lineInfoFactory(src_cloud, *it);
            
            if (pct::LikePowerLine1(ground, line, 10, 0.1, 0.5, 0.5))
            {
                lineClusters.push_back(*it);
                it = jlClusters.erase(it);
                continue;
            }
        }
        ++it;
    }
    std::cout << "电力线识别数量：" << lineClusters.size() << std::endl;

    std::vector<int> indices;
    std::vector<float> sqr_distances;
    for (auto it = jlClusters.begin(); it != jlClusters.end(); )
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr unknowclass(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
        extract.filter(*unknowclass);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> unknowclass_kdtree;
        unknowclass_kdtree.setInputCloud(unknowclass);
        int insrt_size = 0;

        // 遍历每条电力线，是否与当前聚类相距小于一米
        for (int j = 0; j < lineClusters.size(); ++j)
        {
            // 遍历聚类的每一个点
            for (int k = 0; k < lineClusters[j].indices.size(); ++k)
            {
                if (unknowclass_kdtree.radiusSearch(src_cloud->at(lineClusters[j].indices[k]), 1, indices, sqr_distances))
                {
                    insrt_size++;
                    break;
                }
            }
        }
        if (insrt_size >= tower_intersectline_threshold)  // 有3根电力线和他相交了，基本可以确定他就是铁塔了
        {
            towerClusters.push_back(*it);
            it = jlClusters.erase(it);
        }
        else
        {
            ++it;
        }
    }
    pct::MergeTower(src_cloud, ground_indices, jlClusters, lineClusters, towerClusters);
    std::cout << "铁塔识别数量：" << towerClusters.size() << std::endl;
    std::cout << "其他类别数量：" << jlClusters.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tower_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
        extract.filter(*temp_cloud);
        *tower_cloud += *temp_cloud;
    }
    pct::io::save_las(tower_cloud, setting.outputdir + "\\towers.las");
    tower_cloud.reset();

    // 电力线上色
    unsigned int color = setting.cls_intcolor(power_line_str);
    unsigned char r = *(((unsigned char *)&color) + 2);
    unsigned char g = *(((unsigned char *)&color) + 1);
    unsigned char b = *(((unsigned char *)&color) + 0);
    pcl::PointXYZRGB *tmppt;
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  暂时把电力线的点都弄成红色，方便调试
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }
    // 铁塔上色
    color = setting.cls_intcolor(tower_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  暂时把电力线的点都弄成红色，方便调试
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
        std::cout << "检测到铁塔聚类，点数为：" << it->indices.size() << std::endl;
    }
    // 其他点上色
    color = setting.cls_intcolor(veget_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  暂时把电力线的点都弄成红色，方便调试
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }

    pct::io::save_las(src_cloud, outputfile);
    std::cout << "correct end：" << std::endl;
}

