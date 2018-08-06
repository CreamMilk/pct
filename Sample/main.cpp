#include <lasreader.hpp>
#include <laswriter.hpp>
#include <pcl/filters/uniform_sampling.h>   //���Ȳ���
#include <boost/program_options.hpp>


#include <iostream>
#include <string>
#include <io.h>
//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // ���س���

const float LeafSize = 0.3;

void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
void save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);

int main(int argc, char *argv[])
{
    boost::program_options::options_description opts("pointcloud tool sample");
    boost::program_options::variables_map vm;

    opts.add_options()
        ("inputfile", boost::program_options::value<std::string>(), "����·����*.las *.xyz *.ply��")
        ("outputfile", boost::program_options::value<std::string>(), "���·����*.las *.xyz *.ply��")
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
        return EXIT_SUCCESS;
    }
    else
    {
        std::string inputfile, outputfile;
        if (vm.count("inputfile"))
        {
            outputfile = inputfile =  vm["inputfile"].as<std::string>();
        }
        else
        {
            return EXIT_FAILURE;
        }

        if (vm.count("outputfile"))
        {
            outputfile = vm["outputfile"].as<std::string>();
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        Load_las(cloud, inputfile);


        //���Ȳ������Ʋ���ȡ�ؼ���      �����²��������Ĵ���
        pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
        uniform_sampling.setInputCloud(cloud);  //�������
        uniform_sampling.setRadiusSearch(LeafSize);   //���ð뾶 model_ss_��ֵ��0.01����ͨ��agv�޸�
        uniform_sampling.filter(*cloud);   //�˲�

        save_las(cloud, outputfile);
    }

    return EXIT_SUCCESS;
}


void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
{
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(path.c_str());
    if (!lasreadopener.active())
    {
        return;
    }
    LASreader* lasreader = lasreadopener.open();
    cloud->resize(lasreader->header.number_of_point_records);
    size_t step = 0;
    pcl::PointXYZRGB *pt;
    while (lasreader->read_point())
    {
        pt = &cloud->at(step);
        pt->x = lasreader->point.get_x();
        pt->y = lasreader->point.get_y();
        pt->z = lasreader->point.get_z();

        pt->r = lasreader->point.get_R() / (256);
        pt->g = lasreader->point.get_G() / (256);
        pt->b = lasreader->point.get_B() / (256);
        if (pt->r == 0 && pt->g == 0 && pt->b == 0)  // ��Ϊcgal��Ҫ�������ϲ�Ϊ�ڵĵ���ɫ������Ϊlas������ɫ��Ϣ��  ���Բ�ҪĬ��ɫ���óɺ�ɫ��
        {
            pt->r = 1;
            pt->g = 1;
            pt->b = 1;
        }
        ++step;
    }
    lasreader->close();
    delete lasreader;

}

void save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path)
{
    LASheader lasheader;
    lasheader.x_offset = 0;
    lasheader.y_offset = 0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 3;	//1��xyzrgb		,��ɫ���ò��ϣ�δ�ҵ�ԭ����ʱ���ε�
    lasheader.point_data_record_length = 34;

    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(path.c_str());
    if (!laswriteopener.active())
    {
        return;
    }

    // init point 

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    // open laswriter

    LASwriter* laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0)
    {
        fprintf(stderr, "ERROR: could not open laswriter\n");
        return;
    }
    for (auto it = cloud->begin(); it != cloud->end(); ++it)
    {
        laspoint.set_x(it->x);
        laspoint.set_y(it->y);
        laspoint.set_z(it->z);
        laspoint.set_R(it->r * 256);
        laspoint.set_G(it->g * 256);
        laspoint.set_B(it->b * 256);
        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);
    }

    // update the header
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
}
