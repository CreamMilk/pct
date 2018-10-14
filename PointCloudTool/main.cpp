#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <boost/locale.hpp>
#include <chrono>   
#include <Scene_points_with_normal_item.h> //��
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

//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // ���س���
bool ParserCmdline(int argc, char *argv[]);
bool train();
bool classif();
void correct();
void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices>& jlClusters, std::vector < pct::LineInfo>& lineClusters, std::vector <pct::TowerInfo>& towerClusters, std::vector <pct::VegetInfo> &vegetClusters);
bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pct::VegetInfo>& vegetClusters, std::vector <pct::LineInfo>& lineClusters, std::vector <pct::TowerInfo>& towerClusters, double dangerousDistance);

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
        std::cout << "��Ⱥ�����" << std::endl;
        pct::OutlierRemoval(src_cloud);

        // ��Ϊcgal������������ģ������ܰ�ÿ��������ȡ����
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pct::LineInfo> lineClusters;
        std::vector <pct::TowerInfo> towerClusters;
        std::vector <pct::VegetInfo> vegetClusters;
        correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);
        checkLinesDistanceDangerous(src_cloud, ground_indices, vegetClusters, lineClusters, towerClusters, setting.value<float>("dangerdistance"));

        // ����pdf
        std::string pdfexe_path = setting.appdir + "PdfReport\\PdfReport.exe";
        std::string json_path = setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "�����.json";
        if (!boost::filesystem::exists(boost::filesystem::path(pdfexe_path)))
        {
            std::cout << "δ���ҵ� " << pdfexe_path << "������pdfʧ�ܣ�";
            return EXIT_FAILURE;
        }
        else
        {
            std::string cmd_str = pdfexe_path + " --jsonpath " + json_path;
            //system(cmd_str.c_str());
            WinExec(cmd_str.c_str(), SW_HIDE);
            std::cout <<  cmd_str << "\n����pdf�ɹ���";
        }

    }
    else
    {
        return EXIT_FAILURE;
    }
    

    return EXIT_SUCCESS;
}

void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pct::VegetInfo>& vegetClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters,
    double dangerousDistance)
{
    DangerousDistanceCheck ddc;
    ddc.setData(src_cloud 
        , ground_indices
        , vegetClusters
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
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //����ļ��в����ڣ����߲����ļ���
    {
        std::cout << "ѡ��classdir�ļ���·������" << std::endl;
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
        std::cout << "ѡ��inputfile�ļ�·������" << std::endl;
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
        std::cout << "ѡ��outputdirĿ¼����" << std::endl;
        std::cout << outputdir << std::endl;
        return false;
    }

    setsettingitem(classdir, std::string, true, "");
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //����ļ��в����ڣ����߲����ļ���
    {
        std::cout << "�����ļ�����û��ѵ���ļ�config.xml������ѵ��������" << std::endl;
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
    // ����������
    boost::program_options::variables_map vm;
     boost::program_options::options_description opts("pointcloud tool options"
         "\nʾ����"
         "\n��1��" + exe_name + " --cmdtype train --classdir classdir --gridsize 0.3"
         "\n��2��" + exe_name + " --cmdtype classif --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n��3��" + exe_name + " --cmdtype distancecheck --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n����");

    opts.add_options()
        ("cmdtype", boost::program_options::value<std::string>(), "�������ͣ�train classif distancecheck��")
        ("inputfile", boost::program_options::value<std::string>(), "����·����*.las *.xyz *.ply��")
        ("classdir", boost::program_options::value<std::string>(), "�����ļ�Ŀ¼ <dir>")
        ("outputdir", boost::program_options::value<std::string>(), "������Ŀ¼ [dir]")
        ("method", boost::program_options::value<int>(), "���෽�� [int] 0-��ͨ 1-ƽ�� 2-�ǳ�ƽ��")
        ("gridsize", boost::program_options::value<float>(), "Ҷ�ڵ�ߴ� [float]")
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
            std::cout << "����cmdtype�������" << std::endl;
            return false;
        }
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
                    
                    // ��ϡ  
                    std::ostringstream tempfile;
                    tempfile << getpid() << "temp2018.las";
                    pct::simple(laspath, tempfile.str(), setting.gridsize);
                    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));
                    boost::filesystem::remove(boost::filesystem::path(tempfile.str()));

                    std::cout << "�ļ���" << lasname << "..." << std::endl;
                    if (scene_item && scene_item->point_set()->check_colors())
                    {
                        auto start = std::chrono::system_clock::now();
                        Point_set* points = scene_item->point_set();
                        // ��������
                        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
                        classif->compute_features(nb_scales);

                        // ���label
                        classif->add_new_label(classname.c_str());
                        classif->add_new_label(unselect_str);

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


    // ��ϡ  
    std::ostringstream tempfile;
    tempfile << getpid() << "temp2018.las";
    pct::simple(setting.inputfile, tempfile.str(), setting.gridsize);
    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));
    boost::filesystem::remove(boost::filesystem::path(tempfile.str()));

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
            classif->add_new_label(labelname_traverse.c_str(), setting.cls_intcolor(labelname_traverse));
        }

        // ����
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
    std::cout << "��Ⱥ�����" << std::endl;
    //pct::OutlierRemoval(src_cloud);

    // ��Ϊcgal������������ģ������ܰ�ÿ��������ȡ����
    std::vector <pcl::PointIndices> otheCluster;
    std::vector < pct::LineInfo> lineClusters;
    std::vector <pct::TowerInfo> towerClusters;
    std::vector <pct::VegetInfo> vegetClusters;
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);
}

void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& jlClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters,
    std::vector <pct::VegetInfo> &vegetClusters)
{
    const pct::Setting & setting = pct::Setting::ins();
   
    std::string outputfile = setting.outputdir + "\\out.las";
    const int tower_intersectline_threshold = 3;

    std::cout << "correct begin" << std::endl;
    pct::io::save_las(src_cloud, setting.outputdir + "\\cgalclassif.las");


    // ��ȡ���������
    pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
    pct::ExtractGround(src_cloud, cloud_indices, ground_indices);
    std::cout << "�ǵ���㣺" << cloud_indices->indices.size()
        << "����㣺" << ground_indices->indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(ground_indices);
    extract.filter(*ground);


    // �ٶ�ʣ��ĵ���ɫ����
    pct::colorClusters(src_cloud, *cloud_indices, jlClusters);
    std::cout << "����������" << jlClusters.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*src_cloud, *tmpcloud);
    pcl::PointXYZRGB *tmppt1;

    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        unsigned char r = rand() % 256;
        unsigned char g = rand() % 256;
        unsigned char b = rand() % 256;

        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt1 = &tmpcloud->at(*itt);
            tmppt1->r = r;
            tmppt1->g = g;
            tmppt1->b = b;
        }
    }
    for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
    {
        tmppt1 = &tmpcloud->at(*it);
        tmppt1->r = 0;
        tmppt1->g = 0;
        tmppt1->b = 255;
    }
    pct::io::save_las(tmpcloud, setting.outputdir + "\\pcljl.las");
    tmpcloud.reset();

	std::cout << "��������ȡbegin"  << std::endl;
    // ��������ȡ
    // �������Ƿ���70%���ϵĵ����ߵ㣬����ǣ���˵���ǵ����ߵ�
    for (auto it = jlClusters.begin(); it != jlClusters.end(); )
    {
		std::cout << "��������ȡ1" << std::endl;
        if (pct::pointsCountsForColor(src_cloud, *it, setting.cls_intcolor(power_line_str)) > it->indices.size()*0.25)  // power_line":"255, 255, 0
        {
			std::cout << "��������ȡ2" << std::endl;
            pct::LineInfo line = pct::lineInfoFactory(src_cloud, *it);
			std::cout << "��������ȡbegin" << std::endl;
            if (pct::LikePowerLine1(ground, line, 10, 0.1, 0.5, 0.5))
            {
				std::cout << "��������ȡ3" << std::endl;
                lineClusters.push_back(line);
                it = jlClusters.erase(it);
                continue;
            }
			std::cout << "��������ȡ4" << std::endl;
        }
        ++it;
    }
    std::cout << "other������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << std::endl;

    // ������ȡ�����ĳ��������3���������ཻС��2�ף�����Ϊ������
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

        // ����ÿ�������ߣ��Ƿ��뵱ǰ�������С��һ��
        for (int j = 0; j < lineClusters.size(); ++j)
        {
            // ���������ÿһ����
            for (int k = 0; k < lineClusters[j].indices.indices.size(); ++k)
            {
                if (unknowclass_kdtree.radiusSearch(src_cloud->at(lineClusters[j].indices.indices[k]), 2, indices, sqr_distances))
                {
                    insrt_size++;
                    break;
                }
            }
        }
        if (insrt_size >= tower_intersectline_threshold)  // ��3�������ߺ����ཻ�ˣ���������ȷ��������������
        {
            towerClusters.push_back(pct::TowerInfo(*it));
            it = jlClusters.erase(it);
        }
        else
        {
            ++it;
        }
    }
    // �ϲ�����
    std::cout << "�ϲ�����ǰother������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size() << std::endl;
    pct::MergeTower(src_cloud, ground_indices, jlClusters, lineClusters, towerClusters);
    std::cout << "�ϲ�����ǰother������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size() << std::endl;


    // ��������ߺ��������
    for (int i = 0; i < towerClusters.size(); i++)
    {
        pct::getMinMax3D(*src_cloud, towerClusters[i].indices, towerClusters[i].min, towerClusters[i].max);
        pcl::PointXYZRGB midpt;
        
        midpt.x = (towerClusters[i].min.x + towerClusters[i].max.x) / 2;
        midpt.y = (towerClusters[i].min.y + towerClusters[i].max.y) / 2;
        midpt.z = (towerClusters[i].min.z + towerClusters[i].max.z) / 2;
        towerClusters[i].cen = midpt;
    }
    std::cout << "�����������ĵ����" << std::endl;
    pcl::PointXYZRGB temppt;
    pct::TowerInfo tempcluster;
    for (int i = 0; i < (int)towerClusters.size()-1; i++)
    {
        std::cout << "��������" << i << std::endl;
        pcl::PointXYZRGB minpt, maxpt;
        pct::getMinMax3D(*src_cloud, towerClusters[i].indices, minpt, maxpt);
        std::cout << "��ȡ��Χ��" << i << std::endl;
        for (int j = i + 1; j < (int)towerClusters.size() - 1; j++)
        {
            std::cout << "��ȡ��Χ��j" << i << " " << j << std::endl;
            pcl::PointXYZRGB minptj, maxptj;
            pct::getMinMax3D(*src_cloud, towerClusters[j].indices, minptj, maxptj);
            std::cout << "��ȡ��Χ��j" << minptj << std::endl;
            if (minpt.x < minptj.x)
            {
                std::cout << "�ƶ�����j" << i << " " << j << std::endl;
                tempcluster = towerClusters[i];
                towerClusters[i] = towerClusters[j];
                towerClusters[j] = tempcluster;
            }
            std::cout << "�ƶ�����j���" << i << " " << j << std::endl;
        }
        towerClusters[i].tower_no = i+1;
        std::cout << "�����������" << i << std::endl;
    }
    std::cout << "����������" << std::endl;
    if (towerClusters.size())
        towerClusters[(int)towerClusters.size() - 1].tower_no = towerClusters.size();

    for (int i = 0; i < lineClusters.size(); ++i)
    {
        pct::LineInfo &line = lineClusters[i];
        for (int j = 0; j < towerClusters.size(); ++j)
        {
            if (pct::Distance2d(line.sta, towerClusters[j].cen) < 25)
            {
                line.begin_tower_no = towerClusters[j].tower_no;
            }
            else if (pct::Distance2d(line.end, towerClusters[j].cen) < 25)
            {
                line.end_tower_no = towerClusters[j].tower_no;
            }
        }
    }
    std::cout << "�����߱�����" << std::endl;

    //  ��ȡֲ��
    pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
    ground_kdtree.setInputCloud(ground);
    indices.clear();
    sqr_distances.clear();
    for (auto it = jlClusters.begin(); it != jlClusters.end();)
    {
        pct::VegetInfo veg(src_cloud, *it);
        // ��͵�����ظ߶�<10���Ҵ���30���㣬�п�����ֲ�
        if (it->indices.size() > 30 && ground_kdtree.radiusSearch(veg.min.z, 10, indices, sqr_distances) > 0)
        {
            vegetClusters.push_back(veg);
            it = jlClusters.erase(it);
        }
        else
        {
            ++it;
        }
    }
    std::cout << "other������" << jlClusters.size() << "������ʶ��������" << lineClusters.size() << "����ʶ��������" << towerClusters.size() 
        << "ֲ��������" << vegetClusters.size() << "\nһ��" << jlClusters.size() + lineClusters.size() + towerClusters.size() + vegetClusters.size() << std::endl;


    //// ȫ���㶼Ĭ�Ϻ�ɫ
    //for (auto it = src_cloud->begin(); it != src_cloud->end(); ++it)
    //{
    //    it->r = 0;
    //    it->g = 0;
    //    it->b = 0;
    //}

    // �������ɫ
    unsigned int color = setting.cls_intcolor(ground_str);
    unsigned char r = *(((unsigned char *)&color) + 2);
    unsigned char g = *(((unsigned char *)&color) + 1);
    unsigned char b = *(((unsigned char *)&color) + 0);
    pcl::PointXYZRGB *tmppt;
    for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
    {
        tmppt = &src_cloud->at(*it);
        tmppt->r = r;
        tmppt->g = g;
        tmppt->b = b;
    }

    // ��������ɫ
    color = setting.cls_intcolor(power_line_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
   
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }
    // ������ɫ
    color = setting.cls_intcolor(tower_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
        std::cout << "��⵽�������࣬����Ϊ��" << it->indices.indices.size() << std::endl;
    }

    // ֲ������ɫ
    color = setting.cls_intcolor(veget_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }

    // ��������ɫ
    color = setting.cls_intcolor(others_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }

    pct::io::save_las(src_cloud, outputfile);
    std::cout << "correct end��" << std::endl;



    // ��������� .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lines_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        pcl::PointXYZRGB *temp_pt;
        unsigned char r = rand() % 256;
        unsigned char g = rand() % 256;
        unsigned char b = rand() % 256;

        for (int j = 0; j < temp_cloud->size(); ++j)
        {
            temp_pt = &temp_cloud->at(j);
            temp_pt->r = r;
            temp_pt->g = g;
            temp_pt->b = b;
        }

        *lines_cloud += *temp_cloud;
    }
    pct::io::save_las(lines_cloud, setting.outputdir + "\\lines.las");
    lines_cloud.reset();

    // �������� .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tower_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        *tower_cloud += *temp_cloud;
    }
    pct::io::save_las(tower_cloud, setting.outputdir + "\\towers.las");
    tower_cloud.reset();



    // ����vegets .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vegets_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        *vegets_cloud += *temp_cloud;
    }
    pct::io::save_las(vegets_cloud, setting.outputdir + "\\vegets.las");
    vegets_cloud.reset();

    // ����others .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr others_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
        extract.filter(*temp_cloud);
        *others_cloud += *temp_cloud;
    }
    pct::io::save_las(others_cloud, setting.outputdir + "\\others.las");
    others_cloud.reset();

    // ����ground .las
    ground->clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    extract.setInputCloud(src_cloud);
    extract.setIndices(boost::make_shared<std::vector<int>>(ground_indices->indices));
    extract.filter(*ground_cloud);
    pct::io::save_las(ground_cloud, setting.outputdir + "\\ground.las");
    ground_cloud.reset();

}

