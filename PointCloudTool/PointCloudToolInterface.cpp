#define DLL_EXPORTS
#include "PointCloudToolInterface.h"
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
#include <algorithm>  



PointCloudToolInterface::PointCloudToolInterface()
{
}


PointCloudToolInterface::~PointCloudToolInterface()
{
}

ERRORNUM PointCloudToolInterface::train(char *traindir, float gridsize /*= 0.3*/)
{
	srand((unsigned int)time(NULL));
	QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
	vtkOutputWindow::SetGlobalWarningDisplay(0);
	vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8); //1     

	pct::Setting::ins() = pct::Setting();
	pct::Setting& setting = pct::Setting::ins();


	QDir train_dir(QString::fromLocal8Bit(traindir));
	if (!train_dir.exists())
	{
		std::cout << "样本目录打开失败！" << std::endl;
		return ERRORNUM::TRAINFILE;
	}
	setting.classdir = QDir::toNativeSeparators(train_dir.absolutePath()).toLocal8Bit().data();

	setting.gridsize = gridsize;

	::train();

	return ERRORNUM::NORMAL;
}

ERRORNUM PointCloudToolInterface::check_distance_towers(char *lasfile, char *outdir, char *towerdir, char *trainfile, unsigned short geo_zone,
	bool geo_southhemi /*= false*/, int method /*= 2*/, float gridsize /*= 0.3*/, bool overwrite_excel /*= false*/, bool sign_modified /*= true*/)
{
	srand((unsigned int)time(NULL));
	QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
	vtkOutputWindow::SetGlobalWarningDisplay(0);
	vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8); //1     
	
	pct::Setting::ins() = pct::Setting();
	pct::Setting& setting = pct::Setting::ins();


	QFileInfo input_info(QString::fromLocal8Bit(lasfile));
	if(!input_info.exists() || input_info.suffix().toLower() != QStringLiteral("las"))
	{
		std::cout << "点云文件校验失败，请检查路径是否正确！" << std::endl;
		std::cout << lasfile << std::endl;
		return ERRORNUM::LASFILE;
	}
	else
	{
		setting.inputfile = QDir::toNativeSeparators(input_info.absoluteFilePath()).toLocal8Bit().data();
	}

	std::cout << input_info.absoluteFilePath().toLocal8Bit().data() << std::endl;
	std::cout << input_info.absoluteDir().absolutePath().toLocal8Bit().data() << std::endl;
	std::cout << input_info.fileName().toLocal8Bit().data() << std::endl;
	std::cout << (input_info.absoluteDir().absolutePath() + QStringLiteral("/") + input_info.baseName()).toLocal8Bit().data() << std::endl;
	QDir out_info(QString::fromLocal8Bit(outdir));
	if (QString::fromLocal8Bit(outdir).trimmed().isEmpty())
		out_info = QDir((input_info.absoluteDir().absolutePath() + QStringLiteral("/") + input_info.baseName()));
	
	std::cout << out_info.absolutePath().toLocal8Bit().data() << std::endl;
	QString out_qpath = out_info.absolutePath();
	if (!out_info.exists())
	{
		auto start = std::chrono::system_clock::now();
		bool sign = false;
		while (!QDir().mkpath(out_qpath))
		{
			if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 10)
			{
				sign = true;
				break;
			}
		}
		if (sign)
			return ERRORNUM::OUTDIR;
	}
	setting.outputdir = out_info.absolutePath().toLocal8Bit().data();


	QDir tower_info(QString::fromLocal8Bit(towerdir));
	if (!tower_info.exists())
	{
		std::cout << "铁塔目录不存在，请检查路径是否正确！" << std::endl;
		return ERRORNUM::EXCELDIR;
	}
	setting.exceldir = tower_info.absolutePath().toLocal8Bit().data();

	QFileInfo train_info(QString::fromLocal8Bit(trainfile));
	if (!train_info.exists())
	{
		std::cout << "训练文件加载失败！" << std::endl;
		return ERRORNUM::TRAINFILE;
	}
	setting.classdir = QDir::toNativeSeparators(train_info.absoluteDir().absolutePath()).toLocal8Bit().data();

	setting.method = method;

	setting.gridsize = gridsize;

	setting.overrideExcel = overwrite_excel;

	setting.stampcorrectcell = sign_modified;

	setting.zone = geo_zone;

	setting.southhemi = geo_southhemi;



	if (setting.pt.size() == 0)
	{
		return ERRORNUM::LOADSETTING;
	}

	pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
	pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
	std::vector <pcl::PointIndices> otheCluster;
	std::vector <pct::LineInfo> lineClusters;
	std::vector <pct::TowerInfo> towerClusters;
	std::vector <pct::VegetInfo> vegetClusters;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 抽稀  
	pct::simpleAndOutlierRemoval(setting.inputfile, setting.outputdir + "\\simple.las", setting.gridsize, setting.value<int>("simplify"));

	if (setting.pt.get_optional<bool>(pct::to_utf8(pct::String2WString("先分类后提取地面"))).value())
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
		return ERRORNUM::PDFEXPORT;
	}
	else
	{
		std::string cmd_str = pdfexe_path + " --jsonpath " + json_path;
		//system(cmd_str.c_str());
		WinExec(cmd_str.c_str(), SW_HIDE);
		std::cout << cmd_str << "\n导出pdf成功！";
	}

	return ERRORNUM::NORMAL;
}

