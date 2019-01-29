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
#include <QAxObject>
#include <QAxWidget>
#include <QDir>

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
#include "Las2Pnts.h"

//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // 隐藏程序

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8); //1     

    srand((unsigned int)time(NULL));
    pct::Setting & setting = pct::Setting::ins();
    bool priorityClassif = setting.pt.get_optional<bool>(pct::to_utf8(pct::String2WString("先分类后提取地面"))).value();


    std::cout << "priorityClassif = " << priorityClassif << std::endl;
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }
	else
	{
		std::cout << "input：" << setting.inputfile << std::endl;
		std::cout << "output：" << setting.outputdir.c_str() << std::endl;
		std::cout << "output：" << setting.exceldir << std::endl;
	}
    


    if (setting.cmdtype == "train")
    {
        train();
    }
    else if (setting.cmdtype == "classif")
    {
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pct::LineInfo> lineClusters;
        std::vector <pct::TowerInfo> towerClusters;
        std::vector <pct::VegetInfo> vegetClusters;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 抽稀  
        pct::simpleAndOutlierRemoval(setting.inputfile, setting.outputdir + "\\simple.las", setting.gridsize, setting.value<int>("simplify"));
        if (priorityClassif)
        {
            classif();
            pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
            ExtractGround(src_cloud, cloud_indices, ground_indices);
        }
        else
        {
            pct::io::Load_las(src_cloud, setting.outputdir + "\\simple.las");
            ExtractGround(src_cloud, cloud_indices, ground_indices);
            classif();
        }

        // 因为cgal分类结果是整体的，并不能把每个个体提取出来
        pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
        ExtractLinesAndTower(src_cloud, cloud_indices, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);

        SaveTowers(QString::fromLocal8Bit((setting.outputdir + "\\铁塔.xlsx").c_str()) , towerClusters);
        SaveLines(QString::fromLocal8Bit((setting.outputdir + "\\电力线.xlsx").c_str()), lineClusters);
    }
    else if (setting.cmdtype == "distancecheck")
    {
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pct::LineInfo> lineClusters;
        std::vector <pct::TowerInfo> towerClusters;
        std::vector <pct::VegetInfo> vegetClusters;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 抽稀  
        pct::simpleAndOutlierRemoval(setting.inputfile, setting.outputdir + "\\simple.las", setting.gridsize, setting.value<int>("simplify"));

        if (priorityClassif)
        {
            classif();
            pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
            ExtractGround(src_cloud, cloud_indices, ground_indices);
        }
        else
        {
            pct::io::Load_las(src_cloud, setting.outputdir + "\\simple.las");
            ExtractGround(src_cloud, cloud_indices, ground_indices);
            classif();
        }

        pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
        ExtractLinesAndTower(src_cloud, cloud_indices, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);
		setting.tower_excle = QString::fromLocal8Bit((setting.outputdir + "\\铁塔.xlsx").c_str());
		setting.line_excel = QString::fromLocal8Bit((setting.outputdir + "\\电力线.xlsx").c_str());
        SaveTowers(QString::fromLocal8Bit((setting.outputdir + "\\铁塔.xlsx").c_str()), towerClusters);
        SaveLines(QString::fromLocal8Bit((setting.outputdir + "\\电力线.xlsx").c_str()), lineClusters);
        std::cout << "ExtractLinesAndTower end：end end" << std::endl;
        checkLinesDistanceDangerous(src_cloud, ground_indices, vegetClusters, lineClusters, towerClusters);

        // 导出pdf
        std::string pdfexe_path = setting.appdir + "PdfReport\\PdfReport.exe";
        std::string json_path = setting.outputdir + "\\点云检测结果.json";
        if (!boost::filesystem::exists(boost::filesystem::path(pdfexe_path)))
        {
            std::cout << "未能找到 " << pdfexe_path << "，导出pdf失败！";
            return EXIT_FAILURE;
        }
        else
        {
            std::string cmd_str = pdfexe_path + " --jsonpath " + json_path;
            //system(cmd_str.c_str());
            WinExec(cmd_str.c_str(), SW_HIDE);
            std::cout <<  cmd_str << "\n导出pdf成功！";
        }
    }
    else if (setting.cmdtype == "poscorrect")
    {
        if (setting.reclassif)
        {
            pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
            pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
            std::vector <pcl::PointIndices> otheCluster;
            std::vector <pct::LineInfo> lineClusters;
            std::vector <pct::TowerInfo> towerClusters;
            std::vector <pct::VegetInfo> vegetClusters;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            // 抽稀  
            pct::simpleAndOutlierRemoval(setting.inputfile, setting.outputdir + "\\simple.las", setting.gridsize, setting.value<int>("simplify"));

            if (priorityClassif)
            {
                classif();
                pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
                ExtractGround(src_cloud, cloud_indices, ground_indices);
            }
            else
            {
                pct::io::Load_las(src_cloud, setting.outputdir + "\\simple.las");
                ExtractGround(src_cloud, cloud_indices, ground_indices);
                classif();
            }

            pct::io::Load_las(src_cloud, setting.outputdir + "\\classif.las");
            ExtractLinesAndTower(src_cloud, cloud_indices, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);

            QFileInfo inputfile_info(QString::fromLocal8Bit(setting.inputfile.c_str()));
            setting.tower_excle = QDir::toNativeSeparators(inputfile_info.absolutePath() + QStringLiteral("\\") + inputfile_info.baseName() + QStringLiteral("\\铁塔.xlsx"));
			setting.line_excel = QDir::toNativeSeparators(inputfile_info.absolutePath() + QStringLiteral("\\") + inputfile_info.baseName() + QStringLiteral("\\电力线.xlsx"));

			std::cout << "tower_excle" << setting.tower_excle.toLocal8Bit().data() << std::endl;
			std::cout << "line_excel" << setting.line_excel.toLocal8Bit().data() << std::endl;

			SaveTowers(setting.tower_excle, towerClusters);
			SaveLines(setting.line_excel, lineClusters);



			std::cout << "ExtractLinesAndTower end：end end" << std::endl;
			checkLinesDistanceDangerous(src_cloud, ground_indices, vegetClusters, lineClusters, towerClusters);

			// 导出pdf
			std::string pdfexe_path = setting.appdir + "PdfReport\\PdfReport.exe";
			std::string json_path = setting.outputdir + "\\点云检测结果.json";
			if (!boost::filesystem::exists(boost::filesystem::path(pdfexe_path)))
			{
				std::cout << "未能找到 " << pdfexe_path << "，导出pdf失败！";
				return EXIT_FAILURE;
			}
			else
			{
				std::string cmd_str = pdfexe_path + " --jsonpath " + json_path;
				//system(cmd_str.c_str());
				WinExec(cmd_str.c_str(), SW_HIDE);
				std::cout << cmd_str << "\n导出pdf成功！";
			}

			//PositionCorrection(setting.tower_excle);
        }
        else
        {
            //QFileInfo inputfile_info(QString::fromLocal8Bit(setting.inputfile.c_str()));
			//setting.tower_excle = QDir::toNativeSeparators(inputfile_info.absolutePath() + QStringLiteral("\\") + inputfile_info.baseName() + QStringLiteral("\\铁塔.xlsx"));
			//PositionCorrection(setting.tower_excle);
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
    

    return EXIT_SUCCESS;
}
