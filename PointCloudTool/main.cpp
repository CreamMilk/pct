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
#include "dbscan.h"

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif


//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // ���س���
bool ParserCmdline(int argc, char *argv[]);
bool train();
bool classif();
void correct();
void simple(std::string inputfile, std::string outputfile);

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
    correct();

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

        setsettingitem(gridsize, float, false, 0.3);
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


                    //boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(laspath));

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

    //boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(setting.inputfile));

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

void correct()
{
    const pct::Setting & setting = pct::Setting::ins();
    std::string inputfile = setting.outputdir + "\\out.las";
    std::string outputfile = setting.outputdir + "\\out.las";
    const int tower_intersectline_threshold = 3;

    std::cout << "correct begin��" << inputfile << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pct::io::Load_las(src_cloud, inputfile);
    std::cout << "��Ⱥ�����" << std::endl;
    pct::OutlierRemoval(src_cloud);

    // ����
    std::vector <pcl::PointIndices> jlClusters;
    std::vector <pcl::PointIndices> lineClusters;
    std::vector <pcl::PointIndices> towerClusters;
    pct::io::save_las(src_cloud, setting.outputdir + "\\cgalclassif.las");





    std::cout << "��ȡ�����" << std::endl;
    pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    pct::ExtractGround(src_cloud, cloud_indices, ground_indices);
    std::cout << "�ǵ���㣺" << cloud_indices->indices.size()
        << "����㣺" << ground_indices->indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(ground_indices);
    std::cout << "��ȡ����㿪ʼ" << std::endl;
    extract.filter(*ground);
    std::cout << "��ȡ��������" << std::endl;


    struct vec2f {
        float data[2];
        vec2f(float x, float y){ data[0] = x; data[1] = y; };
        float operator[](int idx) const { return data[idx]; }
    };

    auto dbscan = DBSCAN<vec2f, float>();

    auto data = std::vector<vec2f>();
    for (int i = 0; i < cloud_indices->indices.size(); ++i)
    {
        data.push_back(vec2f(src_cloud->at(cloud_indices->indices[i]).x, src_cloud->at(cloud_indices->indices[i]).y));
    }


    //���������ݣ� ά�ȣ���ά���� ���ǰ뾶�� ������С
    dbscan.Run(&data, 2, 1.0f, 30);
    auto noise = dbscan.Noise;
    auto clusters = dbscan.Clusters;

    
    for (auto it = dbscan.Noise.begin(); it != dbscan.Noise.end(); ++it)
    {
        *it = cloud_indices->indices[*it];
    }
    for (auto it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (auto itt = it->begin(); itt != it->end(); ++itt)
        {
            *itt = cloud_indices->indices[*itt];
        }
    }


    std::cout<< "���ܶ���������" <<  clusters.size() << std::endl;
    for (auto it = clusters.begin(); it != clusters.end();)
    {
        if (!pct::likeTower(src_cloud, *it))
        {
            dbscan.Noise.insert(dbscan.Noise.end(), it->begin(), it->end());
            it = clusters.erase(it);
        }
        else
            ++it;
    }

    cloud_indices->indices.clear();
    for (int i = 0; i < dbscan.Noise.size(); ++i)
    {
        cloud_indices->indices.push_back(dbscan.Noise[i]);
    }

    pct::colorClusters(src_cloud, *cloud_indices, jlClusters);
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
    for (auto it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (auto itt = it->begin(); itt != it ->end(); ++itt)
        {
            tmppt1 = &tmpcloud->at(*itt);
            tmppt1->r = 0;
            tmppt1->g = 0;
            tmppt1->b = 255;
        }
    }
    pct::io::save_las(tmpcloud, setting.outputdir + "\\pcljl.las");
    tmpcloud.reset();
   ///////////////
    std::cout << "����������" << jlClusters.size() << std::endl;



    // �������Ƿ���70%���ϵĵ����ߵ㣬����ǣ���˵���ǵ����ߵ�
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

    
    std::cout << "������ʶ��������" << lineClusters.size() << std::endl;


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
            for (int k = 0; k < lineClusters[j].indices.size(); ++k)
            {
                if (unknowclass_kdtree.radiusSearch(src_cloud->at(lineClusters[j].indices[k]), 1, indices, sqr_distances))
                {
                    insrt_size++;
                    break;
                }
            }
        }

        if (insrt_size >= tower_intersectline_threshold)  // ��3�������ߺ����ཻ�ˣ���������ȷ��������������
        {
            towerClusters.push_back(*it);
            jlClusters.erase(it++);
        }
        else
        {
            ++it;
        }
    }
    std::cout << "����ʶ��������" << towerClusters.size() << std::endl;
    std::cout << "�������������" << jlClusters.size() << std::endl;

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

    // ��������ɫ
    pcl::PointXYZRGB *tmppt;
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  ��ʱ�ѵ����ߵĵ㶼Ū�ɺ�ɫ���������
            tmppt->r = 255;
            tmppt->g = 0;
            tmppt->b = 0;
        }
    }
    // ������ɫ
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  ��ʱ�ѵ����ߵĵ㶼Ū�ɺ�ɫ���������
            tmppt->r = 0;
            tmppt->g = 255;
            tmppt->b = 0;
        }

        std::cout << "��⵽�������࣬����Ϊ��" << it->indices.size() << std::endl;
    }
    // ��������ɫ
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);      //  ��ʱ�ѵ����ߵĵ㶼Ū�ɺ�ɫ���������
            tmppt->r = 0;
            tmppt->g = 0;
            tmppt->b = 0;
        }
    }



    pct::io::save_las(src_cloud, outputfile);
    std::cout << "correct end��" << inputfile << std::endl;
}
