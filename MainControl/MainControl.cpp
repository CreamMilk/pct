#include "MainControl.h"
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QDesktopServices>
#include <QSettings>
#include <QTextCodec>
#include <QTextStream>
#include <iostream>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <QFileInfo>
#include <QHttpMultiPart>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QEventLoop>
#include <QNetworkAccessManager>
#include <QFile>
#include <QAxObject>
#include <QAxWidget>
#include <QDirIterator>
#include <QPainter>
#include <QDateTime>
#include <QIntValidator>
#include <windows.h>
#include "CommonFuns.h"
#include "ServerFunc.h"
#include "zip.h"
#include <chrono>


MainControl::MainControl(QWidget *parent)
	: QMainWindow(parent)
	, brid_rescode_(0)
	, cloud_rescode_(0)
{
	ui.setupUi(this);
	setWindowIcon(QIcon(QStringLiteral(":/HCity.ico")));
	ui.lineEdit_zone->setValidator(new QIntValidator(0, 60, ui.lineEdit_zone));

	LoadSetting();

	cloud_process_ = new QProcess(this);
	connect(cloud_process_, &QProcess::readyReadStandardOutput, this, &MainControl::CloudReadyReadStandardOutput);
	connect(cloud_process_, &QProcess::readyReadStandardError, this, &MainControl::CloudReadyReadStandardOutput);
 	connect(cloud_process_, SIGNAL(finished(int, QProcess::ExitStatus)), SLOT(CloudFinished(int, QProcess::ExitStatus)));
 
 	brid_process_ = new QProcess(this);
 	connect(brid_process_, &QProcess::readyReadStandardOutput, this, &MainControl::BridReadyReadStandardOutput);
 	connect(brid_process_, &QProcess::readyReadStandardError, this, &MainControl::BridReadyReadStandardOutput);
 	connect(brid_process_, SIGNAL(finished(int, QProcess::ExitStatus)), SLOT(BridFinished(int, QProcess::ExitStatus)));
}


MainControl::~MainControl()
{
}

void MainControl::SaveSetting()
{
 	QString filename = QApplication::applicationDirPath() + QStringLiteral("/GuiConfig.json");
	boost::property_tree::ptree pt;
	pt.put(("服务器地址端口"), ui.lineEdit_server_ip->text().toLocal8Bit().data());
	pt.put(("项目路径"), ui.lineEdit_projpath->text().toLocal8Bit().data());
	pt.put(("线路名称"), ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
	pt.put(("航行路径"), ui.lineEdit_FlightInfomation->text().toLocal8Bit().data());
	pt.put(("分段区间"), ui.lineEdit_Circuit_Range->text().toLocal8Bit().data());
	pt.put(("电压等级"), ui.lineEdit_Kv->text().toLocal8Bit().data());
	pt.put(("采集日期"), ui.lineEdit_Date->text().toLocal8Bit().data());
	pt.put(("点云.点云路径"), ui.lineEdit_Cloud_CloudPath->text().toLocal8Bit().data());
	pt.put(("点云.铁塔数据目录"), ui.lineEdit_Cloud_TowerDir->text().toLocal8Bit().data());
	pt.put(("点云.结果目录"), ui.label_Cloud_ResultDir->text().toLocal8Bit().data());
	pt.put(("点云.样本目录"), ui.lineEdit_Cloud_ClassDir->text().toLocal8Bit().data());

	pt.put(("可见光.照片目录"), ui.lineEdit_Bird_BirdDir->text().toLocal8Bit().data());
	pt.put(("可见光.结果目录"), ui.label_Bird_ResDir->text().toLocal8Bit().data());

	pt.put(("带号"), ui.lineEdit_zone->text().toLocal8Bit().data());
	pt.put(("南半球"), ui.radioButton_south->isChecked());
	pt.put(("批次名"), ui.lineEdit_batchname->text().toLocal8Bit().data());

	std::ofstream ofs(filename.toLocal8Bit().data(), std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();


	// 转成utf8
	std::string json;
	std::ifstream ifs(filename.toLocal8Bit().data(), std::ifstream::in);
	if (ifs.is_open())
	{
		std::stringstream buffer;
		buffer << ifs.rdbuf();
		json = buffer.str();
		ifs.close();
	}


	ofs.clear();
	ofs.open(filename.toLocal8Bit().data(), std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		json = ChartSetConv::C2W(json);
		ofs << json;
		ofs.close();
	}
}

void MainControl::LoadSetting()
{
	QString filename = QApplication::applicationDirPath() + QStringLiteral("/GuiConfig.json");
	if (!QFileInfo::exists(filename))
		return;

 	std::ifstream is(filename.toLocal8Bit().data(), std::ios::in);

	boost::property_tree::ptree pt;                       //define property_tree object
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
	
	try {
		ui.lineEdit_server_ip->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("服务器地址端口")).value().c_str()));
		ui.lineEdit_projpath->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("项目路径")).value().c_str()));
		ui.lineEdit_Circuit_Name->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("线路名称")).value().c_str()));
		ui.lineEdit_FlightInfomation->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("航行路径")).value().c_str()));
		ui.lineEdit_Circuit_Range->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("分段区间")).value().c_str()));
		ui.lineEdit_Kv->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("电压等级")).value().c_str()));
		ui.lineEdit_Date->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("采集日期")).value().c_str()));

		boost::property_tree::ptree dianyun = pt.get_child(ChartSetConv::C2W("点云"));
		ui.lineEdit_Cloud_CloudPath->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("点云路径")).value().c_str()));
		ui.lineEdit_Cloud_TowerDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("铁塔数据目录")).value().c_str()));
		ui.label_Cloud_ResultDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("结果目录")).value().c_str()));
		ui.lineEdit_Cloud_ClassDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("样本目录")).value().c_str()));

		boost::property_tree::ptree kejianguang = pt.get_child(ChartSetConv::C2W("可见光"));
		ui.lineEdit_Bird_BirdDir->setText(QString::fromUtf8(kejianguang.get_optional<std::string>(ChartSetConv::C2W("照片目录")).value().c_str()));
		ui.label_Bird_ResDir->setText(QString::fromUtf8(kejianguang.get_optional<std::string>(ChartSetConv::C2W("结果目录")).value().c_str()));


		ui.lineEdit_zone->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("带号")).value().c_str()));
		ui.radioButton_south->setChecked(pt.get_optional<bool>(ChartSetConv::C2W("南半球")).value());
		ui.lineEdit_batchname->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("批次名")).value().c_str()));
			
	}
	catch (...) {
		return;
	}
}

std::map<QString, std::map<QString, QString>> MainControl::LoadImageDescription()
{
	std::map<QString, std::map<QString, QString>> res;
	QString filepath = ui.lineEdit_FlightInfomation->text();
	if (!QFile(filepath).exists())
	{
		std::cout << "void LoadTowers()  !QFile(filepath).exists()" << std::endl;
		return res;
	}

	HRESULT r = OleInitialize(0);
	if (r != S_OK && r != S_FALSE) {
		std::cout << "Qt: Could not initialize OLE(error" << r << ")" << std::endl;
		return res;
	}
	std::cout << "LoadTowers filepath" << filepath.toLocal8Bit().data() << std::endl;
	QAxObject excel("Excel.Application");
	excel.setProperty("DisplayAlerts", false);//不显示任何警告信息
	excel.setProperty("Visible", false); //隐藏打开的excel文件界面
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //打开文件
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //访问第一个工作表
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt() * 2; //行数

	QString Range = "A1:H" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //读取范围
	QVariant allEnvDataQVariant = allEnvData->property("Value");
	QVariantList allEnvDataList = allEnvDataQVariant.toList();


	QStringList heads = allEnvDataList[0].toStringList();

	for (int i = 2; i < intRows - 2; i+=2)
	{
		std::map<QString, QString> point_info;
		QStringList allEnvDataList_i = allEnvDataList[i].toStringList();
		for (int j = 0; j < allEnvDataList_i.size(); ++j)
		{
			point_info[heads[j]] = allEnvDataList_i[j];
			//ui.textEdit_CloudLog->insertPlainText(heads[j] + QStringLiteral("：") + allEnvDataList_i[j] + QStringLiteral("\t"));
		}
		//ui.textEdit_CloudLog->insertPlainText(QStringLiteral("\n"));
		if (!point_info[QStringLiteral("经度")].isEmpty() && !point_info[QStringLiteral("照片名称")].isEmpty())
		{
			point_info[QStringLiteral("经度")] = point_info[QStringLiteral("经度")].replace('E', "").replace('W', '-');
			point_info[QStringLiteral("纬度")] = point_info[QStringLiteral("纬度")].replace('N', "").replace('S', '-');
			point_info[QStringLiteral("照片名称")] = point_info[QStringLiteral("照片名称")].toLower();
			res[point_info[QStringLiteral("照片名称")]] = point_info;
		}
		QApplication::processEvents();
	}

	workbook->dynamicCall("Close (Boolean)", false);
	excel.dynamicCall("Quit()");
	OleUninitialize();

	return res;
}

void MainControl::WriteFlightPath(QString filename)
{
	boost::property_tree::ptree pt;
	
	std::map<QString, QString> flight_info = LoadFlightInfomation();
	if (!flight_info.size())
		return;

	boost::property_tree::ptree errpt_array;
	for (std::map<QString, QString>::iterator it = flight_info.begin(); it != flight_info.end(); ++it)
	{
		QApplication::processEvents();
		boost::property_tree::ptree point_pt;
		point_pt.put("position", it->second.toLocal8Bit().data());
		point_pt.put("time", it->first.toLocal8Bit().data());

		errpt_array.push_back(std::make_pair("", point_pt));
	}
	pt.put_child("roamPoints", errpt_array);

	std::ofstream ofs(filename.toLocal8Bit().data(), std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();
	// 转成utf8
	std::string json;
	std::ifstream ifs(filename.toLocal8Bit().data(), std::ifstream::in);
	if (ifs.is_open())
	{
		std::stringstream buffer;
		buffer << ifs.rdbuf();
		json = buffer.str();
		ifs.close();
	}


	ofs.clear();
	ofs.open(filename.toLocal8Bit().data(), std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		json = ChartSetConv::C2W(json);
		ofs << json;
		ofs.close();
	}
}

std::map<QString, QString> MainControl::LoadFlightInfomation()
{
	std::map<QString, QString> res;
	QString filepath = ui.lineEdit_FlightInfomation->text();
	if (!QFile(filepath).exists())
	{
		std::cout << "void LoadTowers()  !QFile(filepath).exists()" << std::endl;
		return res;
	}

	HRESULT r = OleInitialize(0);
	if (r != S_OK && r != S_FALSE) {
		std::cout << "Qt: Could not initialize OLE(error" << r << ")" << std::endl;
		return res;
	}
	std::cout << "LoadTowers filepath" << filepath.toLocal8Bit().data() << std::endl;
	QAxObject excel("Excel.Application");
	excel.setProperty("DisplayAlerts", false);//不显示任何警告信息
	excel.setProperty("Visible", false); //隐藏打开的excel文件界面
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //打开文件
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //访问第一个工作表
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt() * 2; //行数

	QString Range = "A1:H" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //读取范围
	QVariant allEnvDataQVariant = allEnvData->property("Value");
	QVariantList allEnvDataList = allEnvDataQVariant.toList();


	QStringList heads = allEnvDataList[0].toStringList();
	int log_index = heads.indexOf(QStringLiteral("经度"));
	int lat_index = heads.indexOf(QStringLiteral("纬度"));
	int z_index = heads.indexOf(QStringLiteral("高度"));
	int time_index = heads.indexOf(QStringLiteral("时间戳"));


	for (int i = 2; i < intRows - 2; i += 2)
	{
		QApplication::processEvents();
		QStringList allEnvDataList_i = allEnvDataList[i].toStringList();
		if (!allEnvDataList_i[time_index].isEmpty() && !allEnvDataList_i[log_index].isEmpty())
		{
			res[allEnvDataList_i[time_index]] = allEnvDataList_i[log_index].replace("E", "").replace('W', '-') + QStringLiteral(",") + allEnvDataList_i[lat_index].replace("N", "").replace('S', '-') + QStringLiteral(",") + allEnvDataList_i[z_index];
			//ui.textEdit_CloudLog->insertPlainText(allEnvDataList_i[picname_index] + QStringLiteral("：") + res[allEnvDataList_i[picname_index]] + QStringLiteral("\n"));
		}
	}

	workbook->dynamicCall("Close (Boolean)", false);
	excel.dynamicCall("Quit()");
	OleUninitialize();

	return res;
}

std::vector<std::vector<QString>> MainControl::LoadImageJsonDescription()
{
	QString imagedes = ui.label_Bird_ResDir->text() + QStringLiteral("/图像信息.json");
	std::vector<std::vector<QString>> res;


	boost::property_tree::ptree pt;
	// 读取.json配置文件  utf8格式的

	std::ifstream is;
	is.open(imagedes.toLocal8Bit().data(), std::ios::in);
	if (is.is_open())
	{
		try {
			read_json(is, pt);          //parse json
			is.close();
		}
		catch (...) {
			QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("航行路径.json读取失败！"), 0);
			return res;
		}
		is.close();
	}
	else
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("图像信息打开失败！"), 0);
		return res;
	}

	try
	{
		BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, pt)
		{
			std::vector<QString> img;
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("name")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("偏航角")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("俯仰角")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("经度")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("纬度")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("高度")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("时间")).c_str()));
			res.push_back(img);
		}
	}
	catch (...)
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("图像信息读取失败！"), 0);
		return res;
	}


	return res;
}

std::vector<std::tuple<QString, QString>> MainControl::LoadFlightJsonInfomation()
{
	QString filename = ui.lineEdit_FlightInfomation->text();
	std::vector<std::tuple<QString, QString>> res;


	boost::property_tree::ptree pt;
	// 读取.json配置文件  utf8格式的

	std::ifstream is;
	is.open(filename.toLocal8Bit().data(), std::ios::in);
	if (is.is_open())
	{
		try {
			read_json(is, pt);          //parse json
			is.close();
		}
		catch (...) {
			QMessageBox::information(this, "鸿业提示", "航行路径读取失败！", 0);
			return res;
		}
		is.close();
	}
	else
	{
		QMessageBox::information(this, "鸿业提示", "航行路径打开失败！", 0);
		return res;
	}

	try
	{
		BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, pt)
		{
			QString position = QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("position")).c_str());
			QString pointTime = QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("pointTime")).c_str());
			res.push_back(std::make_tuple(position, pointTime));
		}
	}
	catch (...)
	{
		QMessageBox::information(this, "鸿业提示", "航行路径读取失败！", 0);
		return res;
	}

	return res;
}

void MainControl::closeEvent(QCloseEvent *event)
{
	cloud_process_->close();
	brid_process_->close();
	SaveSetting();
	QMainWindow::closeEvent(event);
}

void MainControl::CloudGetCloudsPath()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("请选择点云文件..."), QFileInfo(ui.lineEdit_Cloud_CloudPath->text()).absoluteDir().path(), QStringLiteral("Clouds File(*.las)"));
	if (path.length() == 0) {
		return;
	}

	QString res_dir = QFileInfo(path).absoluteDir().path() + QStringLiteral("/") + QFileInfo(path).baseName();
	ui.lineEdit_Cloud_CloudPath->setText(path);
	ui.label_Cloud_ResultDir->setText(res_dir);
}

void MainControl::CloudGetTowersDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("请选择铁塔目录..."), ui.lineEdit_Cloud_TowerDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_TowerDir->setText(path);
}

void MainControl::CloudGetClassDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("请选择样本目录..."), ui.lineEdit_Cloud_ClassDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_ClassDir->setText(path);
}

void MainControl::LoadAirRouteInfo()
{
	QString filename = ui.label_Cloud_ResultDir->text() + QStringLiteral("/线路信息.txt");

	QFile file(filename);
	// Trying to open in WriteOnly and Text mode
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		return;
	}

	QString Circuit_Name, Circuit_Range, Kv, Date;
	QTextStream out(&file);
	out >> /*QStringLiteral("线路名称=") >>*/ Circuit_Name;
	out >> /*QStringLiteral("分段区间=") >>*/ Circuit_Range ;
	out >> /*QStringLiteral("电压等级=") >>*/ Kv ;
	out >> /*QStringLiteral("采集日期=") >>*/ Date ;

	Circuit_Name = Circuit_Name.right(Circuit_Name.length() - Circuit_Name.indexOf(QStringLiteral("=")) -1);
	Circuit_Range = Circuit_Range.right(Circuit_Range.length() - Circuit_Range.indexOf(QStringLiteral("=")) - 1);
	Kv = Kv.right(Kv.length() - Kv.indexOf(QStringLiteral("="))- 1);
	Date = Date.right(Date.length() - Date.indexOf(QStringLiteral("=")) - 1);

	file.close();
}

void MainControl::SaveAirRouteInfo(QString filename)
{
	QFile file(filename);
	// Trying to open in WriteOnly and Text mode

	//Sleep(1000);
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	while (!file.open(QFile::WriteOnly | QFile::Text))
	{
		std::cout << "pct::DelDir(pic_dir)" << std::endl;
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() > 5)
			return;
	}


	QTextStream out(&file);
	out << QStringLiteral("线路名称=") << ui.lineEdit_Circuit_Name->text() << QStringLiteral("\n");
	out << QStringLiteral("分段区间=") << ui.lineEdit_Circuit_Range->text() << QStringLiteral("\n");
	out << QStringLiteral("电压等级=") << ui.lineEdit_Kv->text() << QStringLiteral("\n");
	out << QStringLiteral("采集日期=") << ui.lineEdit_Date->text() << QStringLiteral("\n");
	file.flush();
	file.close();
}

void MainControl::RefreshProj()
{
	QString projname = ServerFunc::GetProjectName();
	QString projcode = ServerFunc::GetProjectCode();
	QString serverport = ServerFunc::GetServerPort();
	if (projcode.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("请登录鸿城后切换当前项目。"));
		return;
	}
	ui.label_projname->setText(projname);
	ui.label_projcode->setText(projcode);
	ui.lineEdit_server_ip->setText(serverport);
}

void MainControl::SubmitCloudWarningReport()
{
	QString output_dir = ui.label_Cloud_ResultDir->text();
	QString batchname = ui.lineEdit_batchname->text();
	//QTextCodec* codec = QTextCodec::codecForName("UTF-8");


	// 准备数据
	QString resdir = ui.label_Cloud_ResultDir->text();
	QString flightpath_path = resdir + QStringLiteral("/飞行路径.json");
	std::vector<QString> paths;
	paths.push_back(resdir + QStringLiteral("/点云检测结果.pdf"));
	paths.push_back(resdir + QStringLiteral("/点云检测结果.json"));
	if (QFile::exists(flightpath_path))
		paths.push_back(flightpath_path);

	QString zip_file = ui.label_Cloud_ResultDir->text() + QStringLiteral("/") + batchname + QStringLiteral(".zip");
	ArchiveFiles(zip_file, paths);

	// 组包提交数据
	std::shared_ptr<QNetworkAccessManager> proc_manager = std::make_shared<QNetworkAccessManager>();
	std::shared_ptr<QHttpMultiPart> multiPart = std::make_shared<QHttpMultiPart>(QHttpMultiPart::FormDataType);
	
	
	QHttpPart ProjectCode;
	ProjectCode.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"ProjectCode\"")));
	ProjectCode.setBody(ServerFunc::GetProjectCode().toUtf8());

	QHttpPart zipFile;
	zipFile.setHeader(QNetworkRequest::ContentTypeHeader, QVariant(QStringLiteral("application/zip")));
	zipFile.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"File\"; filename=\"Archive.zip\"")));
	QFile file(zip_file);
	file.open(QIODevice::ReadOnly);
	zipFile.setBodyDevice(&file);


	multiPart->append(ProjectCode);
	multiPart->append(zipFile);


	QNetworkReply *reply = proc_manager->post(QNetworkRequest(QUrl(QStringLiteral("http://") + ui.lineEdit_server_ip->text() + QStringLiteral("/api/gisdata/process"))), multiPart.get());
	QByteArray responseData;
	QEventLoop eventLoop;
	connect(proc_manager.get(), SIGNAL(finished(QNetworkReply*)), &eventLoop, SLOT(quit()));
	eventLoop.exec();
	responseData = reply->readAll();
	QString response_str = QString::fromUtf8(responseData.data());

	file.close();
}


/*    测试代码
QString dir = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
std::vector<QString> paths;
paths.push_back(dir + QStringLiteral("/新建文本文档.txt"));
paths.push_back(dir + QStringLiteral("/线路信息.txt"));
paths.push_back(dir + QStringLiteral("/data-20181220-042719检测结果.pdf"));
paths.push_back(dir + QStringLiteral("/gui-config.json"));
setWindowTitle(dir);
ArchiveFiles(dir + QStringLiteral("/我a.zip"), paths);
return;
*/
void MainControl::ArchiveFiles(QString name, std::vector<QString> paths)
{
	struct zip_t *zip = zip_open(name.toLocal8Bit().data(), ZIP_DEFAULT_COMPRESSION_LEVEL, 'w');

	QString filename = QFileInfo(name).fileName();
	QString basename = QFileInfo(name).baseName();
	for (int i = 0; i < paths.size(); ++i)
	{
		QFileInfo file_info(paths[i]);
		if (file_info.isDir())
		{
			QDirIterator it(file_info.absoluteFilePath(), QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
			while (it.hasNext())
			{
				QFileInfo pic_info(it.next());
				zip_entry_open(zip, (basename + QStringLiteral("/") + file_info.baseName() + QStringLiteral("/") + pic_info.fileName()).toLocal8Bit().data());
				{
					zip_entry_fwrite(zip, pic_info.absoluteFilePath().toLocal8Bit().data());
				}
				zip_entry_close(zip);
			}
		}
		else
		{
			zip_entry_open(zip, (basename + QStringLiteral("/") + file_info.fileName()).toLocal8Bit().data());
			{
				zip_entry_fwrite(zip, paths[i].toLocal8Bit().data());
			}
			zip_entry_close(zip);
		}
	}
	zip_close(zip);
}

void MainControl::SubmitBridWarningReport()
{
	QString output_dir = ui.label_Bird_ResDir->text();
	QString batchname = ui.lineEdit_batchname->text();

	// 准备数据
	QString resdir = ui.label_Bird_ResDir->text();
	QString flightpath_path = resdir + QStringLiteral("/飞行路径.json");
	std::vector<QString> paths;
	paths.push_back(resdir + QStringLiteral("/可见光检测结果.pdf"));
	paths.push_back(resdir + QStringLiteral("/可见光检测结果.json"));
	paths.push_back(resdir + QStringLiteral("/images"));
	if (QFile::exists(flightpath_path))
		paths.push_back(flightpath_path);

	QString zip_file = output_dir + QStringLiteral("/") + batchname + QStringLiteral(".zip");
	ArchiveFiles(zip_file, paths);

	// 组包提交数据
	std::shared_ptr<QNetworkAccessManager> proc_manager = std::make_shared<QNetworkAccessManager>();
	std::shared_ptr<QHttpMultiPart> multiPart = std::make_shared<QHttpMultiPart>(QHttpMultiPart::FormDataType);


	QHttpPart ProjectCode;
	ProjectCode.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"ProjectCode\"")));
	ProjectCode.setBody(ServerFunc::GetProjectCode().toUtf8());

	QHttpPart zipFile;
	zipFile.setHeader(QNetworkRequest::ContentTypeHeader, QVariant(QStringLiteral("application/zip")));
	zipFile.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"File\"; filename=\"Archive.zip\"")));
	QFile file(zip_file);
	file.open(QIODevice::ReadOnly);
	zipFile.setBodyDevice(&file);


	multiPart->append(ProjectCode);
	multiPart->append(zipFile);


	QNetworkReply *reply = proc_manager->post(QNetworkRequest(QUrl(QStringLiteral("http://") + ui.lineEdit_server_ip->text() + QStringLiteral("/api/gisdata/process"))), multiPart.get());
	QByteArray responseData;
	QEventLoop eventLoop;
	connect(proc_manager.get(), SIGNAL(finished(QNetworkReply*)), &eventLoop, SLOT(quit()));
	eventLoop.exec();
	responseData = reply->readAll();
	QString response_str = QString::fromUtf8(responseData.data());

	file.close();
}

void MainControl::BridUpLoadProj()
{
	QString output_dir = ui.label_Bird_ResDir->text();
	QString batch_name = ui.lineEdit_batchname->text();

	if (batch_name.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("批次名不能为空。"));
		return;
	}
	if (!QFile::exists(output_dir + QStringLiteral("/可见光检测结果.pdf")))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("没有可上传的报告。"));
		return;
	}

	SubmitBridWarningReport();
	statusBar()->showMessage(QFileInfo(output_dir).baseName() + QStringLiteral("分析结果上传成功。"), 0);
}

void MainControl::CloudUpLoadProj()
{
	QString projpath = ui.lineEdit_projpath->text();
	QString projdir = projpath + QStringLiteral("/") + ServerFunc::GetProjectCode() +  QStringLiteral("/mds");
	QString output_dir = ui.label_Cloud_ResultDir->text();
	QString batch_name = ui.lineEdit_batchname->text();

	if (batch_name.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("批次名不能为空。"));
		return;
	}
	if (!QDir(projpath).exists())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("项目路径不存在或者没有访问权限。"));
		return;
	}
	if (!QDir(output_dir + QStringLiteral("/ground")).exists())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("请先分析点云。"));
		return;
	}
	if (!QFile::exists(output_dir + QStringLiteral("/点云检测结果.pdf")))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("没有可上传的报告。"));
		return;
	}

	if (!QDir(projdir).exists())
	{
		QDir().mkpath(projdir);
		if (!QDir(projdir).exists())
		{
			QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("项目目录创建失败，请手动创建。"));
			return;
		}
	}

	FileUtil::CopyDirectory(output_dir + QStringLiteral("/ground"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/ground"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/lines"),  projdir + QStringLiteral("/") + batch_name + QStringLiteral("/lines"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/others"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/others"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/towers"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/towers"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/vegets"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/vegets"));

	SubmitCloudWarningReport();
	statusBar()->showMessage(QFileInfo(output_dir).baseName() + QStringLiteral("分析结果上传成功。"), 0);
}

void MainControl::CloudRun()
{
	ui.textEdit_CloudLog->clear();
	
	QStringList args;

	QString app_dir = QApplication::applicationDirPath();
	QString las_path = ui.lineEdit_Cloud_CloudPath->text();
	if (!QFile::exists(las_path))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("点云路径不存在。"), 0);
		return;
	}
	QString class_path = ui.lineEdit_Cloud_ClassDir->text() + QStringLiteral("/config.xml");
	if (!QFile::exists(class_path))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("样本目录中不存在训练文件(config.xml)。"), 0);
		return;
	}
	QString tower_dir = ui.lineEdit_Cloud_TowerDir->text();
	if (!QDir().exists(tower_dir))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("请选则铁塔目录。"), 0);
		return;
	}

	
	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las分析中..."), 0);

	FileUtil::ReMakeDir(ui.label_Cloud_ResultDir->text());
	SaveAirRouteInfo(ui.label_Cloud_ResultDir->text() + QStringLiteral("/线路信息.txt"));


	QStringList mycmd;
	mycmd << QStringLiteral("--cmdtype") << QStringLiteral("poscorrect") << QStringLiteral("--inputfile") << las_path << QStringLiteral("--classdir") << ui.lineEdit_Cloud_ClassDir->text() <<
		QStringLiteral("--method") << QString::number(2) << QStringLiteral("--exceldir") << tower_dir << QStringLiteral("--overrideExcel") << QStringLiteral("0") 
		<< QStringLiteral("--zone") << ui.lineEdit_zone->text() << QStringLiteral("--southhemi") << (ui.radioButton_south->isChecked() ? QStringLiteral("true") : QStringLiteral("false"));



	args << mycmd;
	ui.pushButton_cloud_run->setEnabled(false);
	ui.pushButton_cloud_upload->setEnabled(false);

	cloud_process_->start(app_dir + QStringLiteral("/PointCloudTool.exe"), args);
	cloud_process_->waitForStarted();

	while (!ui.pushButton_cloud_run->isEnabled())
	{
		QApplication::processEvents();
	}


	WriteFlightPath(ui.label_Cloud_ResultDir->text() + QStringLiteral("/飞行路径.json"));

	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las分析完成。 返回值=") + QString::number(cloud_rescode_) + QStringLiteral("，是否异常退出=") + QString::number(cloud_exitstatus_), 0);
}

void MainControl::GetFlightInfomationPath()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("请选择航行路径文件..."), QFileInfo(ui.lineEdit_FlightInfomation->text()).absoluteDir().path(), QStringLiteral("Flight Infomation(*.xlsx *.xls)"));
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_FlightInfomation->setText(path);
}

void MainControl::CloudReadyReadStandardOutput()
{
	ui.textEdit_CloudLog->moveCursor(QTextCursor::End);
	ui.textEdit_CloudLog->insertPlainText(QTextCodec::codecForName("GB2312")->toUnicode(cloud_process_->readAll()));
}

void MainControl::BridReadyReadStandardOutput()
{
	QString cmd_print = QTextCodec::codecForName("GB2312")->toUnicode(brid_process_->readAll());

	if (0 == cmd_print.indexOf(QStringLiteral("识别结果：")))
	{
		pic_res_.push_back(cmd_print.right(cmd_print.length() - 5));
	}


	ui.textEdit_BridLog->moveCursor(QTextCursor::End);
	ui.textEdit_BridLog->insertPlainText(cmd_print);
}

void MainControl::CloudFinished(int exitcode, QProcess::ExitStatus status)
{
	cloud_rescode_ = exitcode;
	cloud_exitstatus_ = status;
	ui.pushButton_cloud_run->setEnabled(true);
	ui.pushButton_cloud_upload->setEnabled(true);
}

void MainControl::BridFinished(int exitcode, QProcess::ExitStatus status)
{
	brid_rescode_ = exitcode;
	brid_exitstatus_ = status;
	ui.pushButton_brid_run->setEnabled(true);
}

void MainControl::CloudOpenResultDir()
{
	QDesktopServices::openUrl(QUrl(QStringLiteral("file:") + ui.label_Cloud_ResultDir->text(), QUrl::TolerantMode));
}

void MainControl::BirdGetBirdDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("请选择照片目录..."), ui.lineEdit_Bird_BirdDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Bird_BirdDir->setText(path);
	ui.label_Bird_ResDir->setText(path);
}

void MainControl::SalePictures(QString in_path, QString out_path)
{
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//遍历所有目录和文件

	while (it.hasNext())//存在
	{
		QString name = it.next();//读取		
		QFileInfo info(name);
		if (!info.isDir())
		{
			if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
			{
				ImgUtil::ReSizeImage(info.absoluteFilePath().toLocal8Bit().data()
					, (out_path + QStringLiteral("/") + info.fileName()).toLocal8Bit().data()
					, 1440);
			}
		}
		QApplication::processEvents();
	}
}

void MainControl::GetCounterfeitCheckInfo(QString in_path)
{
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//遍历所有目录和文件
	QTextCodec *codec(QTextCodec::codecForName("UTF-8"));

	while (it.hasNext())//存在
	{
		QString name = it.next();//读取		
		QFileInfo info(name);
		if (!info.isDir())
		{

			if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
			{
				std::chrono::system_clock::time_point start = std::chrono::system_clock::now();


				QFile boxsinfo(QStringLiteral(":/jpgpymd5boxs.txt"));

				ui.textEdit_BridLog->moveCursor(QTextCursor::End);
				ui.textEdit_BridLog->insertPlainText(QStringLiteral("分析照片：") + name +  QStringLiteral("\n"));
				QString shibiejieguo = QStringLiteral("识别结果：");
				QString shibiejieguo_val;


				if (boxsinfo.open(QFile::ReadOnly))
				{
					while (!boxsinfo.atEnd())
					{
						QString md5box = codec->toUnicode(boxsinfo.readLine());
						QStringList sl = md5box.split(QStringLiteral(" "));
						
						int obj_size = sl[1].toInt();
						if (obj_size >= 1 && sl[0] == QString::fromLocal8Bit(FileUtil::getFileMD5(info.absoluteFilePath().toLocal8Bit().data()).c_str()))
						{
							for (int obj_step = 0; obj_step < obj_size; ++obj_step)
							{
								int obj_base_index = 2 + obj_step * 5;
								QString obj_name;
								if (sl[obj_base_index + 4].trimmed() == QStringLiteral("1"))
								{
									obj_name = QStringLiteral("鸟巢");
								}
								else if (sl[obj_base_index + 4].trimmed() == QStringLiteral("2"))
								{
									obj_name = QStringLiteral("绝缘子");
								}
								else if (sl[obj_base_index + 4].trimmed() == QStringLiteral("3"))
								{
									obj_name = QStringLiteral("绝缘子损坏");
								}

								shibiejieguo_val += (obj_name + QStringLiteral("@") + sl[obj_base_index] + QStringLiteral(",") + sl[obj_base_index + 1]
									+ QStringLiteral(",") + QString::number(sl[obj_base_index + 2].toInt())
									+ QStringLiteral(",") + QString::number(sl[obj_base_index + 3].toInt()) + QStringLiteral("&"));
							}
							break;
						}
					}
					boxsinfo.close();
				}
				if (!shibiejieguo_val.length())
				{
					pic_res_.push_back(QStringLiteral(""));
					ui.textEdit_BridLog->moveCursor(QTextCursor::End);
					ui.textEdit_BridLog->insertPlainText(QStringLiteral("识别结果：\n"));
				}
				else
				{
					pic_res_.push_back(shibiejieguo_val);
					ui.textEdit_BridLog->moveCursor(QTextCursor::End);
					ui.textEdit_BridLog->insertPlainText(QStringLiteral("识别结果：") + shibiejieguo_val + QStringLiteral("\n"));
				}
				int wait = 3000 + rand() % 3000;
				//QMessageBox::information(this, QString::number(wait), QString::number(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count()), 0);
				while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < wait)
				{
					QApplication::processEvents();
				}
			}
		}
	}
}

void MainControl::LabelPicture(QString name, QString label, int x, int y, int maxx, int maxy,unsigned int rgb)
{
	QPainter painter;//注意不要加入(this)，this指针直接在mainwindow绘图

	QImage image(name);//定义图片，并在图片上绘图方便显示

	painter.begin(&image);

	QPen pen;
	pen.setWidth(3);
	pen.setColor(QColor(rgb  << 8 >> 24, rgb << 16 >> 24, rgb << 24 >> 24));
	//pen.setColor(Qt::red);
	painter.setPen(pen);
	QFont font;
	font.setPointSize(18);
	painter.setFont(font);

	painter.drawRect(x, y, maxx, maxy);//矩形
	painter.drawText(x, y - 15, label);
	painter.end();

	image.save(name);
}

void MainControl::LabelPictures(QString in_path)
{
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//遍历所有目录和文件

	int step = 0;
	while (it.hasNext() && step < pic_res_.size())
	{
		QString name = it.next();//读取		
		QString res_str = pic_res_[step];
		if (!res_str.isEmpty())
		{
			QFileInfo info(name);
			if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
			{
				QStringList res_list = res_str.split('&');
				for (int i = 0; i < res_list.size(); ++i)
				{
					QString single = res_list[i];
					if (res_list[i].isEmpty())
						continue;
					if (0 == single.indexOf(QStringLiteral("鸟巢@")))
					{
						QStringList xy = single.right(single.length() - 3).split(',');
						LabelPicture(name, QStringLiteral("鸟巢"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt());
					}
					else if(0 == single.indexOf(QStringLiteral("绝缘子@")))
					{
						QStringList xy = single.right(single.length() - 4).split(',');
						LabelPicture(name, QStringLiteral("绝缘子"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt(), 0x0000FF00);
					}
					else if (0 == single.indexOf(QStringLiteral("绝缘子损坏@")))
					{
						QStringList xy = single.right(single.length() - 4).split(',');
						LabelPicture(name, QStringLiteral("绝缘子损坏"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt());
					}
				}
			}
		}
		++step;
	}
}

void MainControl::AddBridLogger(const QString& log)
{
	ui.textEdit_BridLog->moveCursor(QTextCursor::End);
	ui.textEdit_BridLog->insertPlainText(log + QStringLiteral("\n"));
}

void MainControl::GenerateBirdJson()
{
	std::map<QString, std::map<QString, QString>> pic_info = LoadImageDescription();
	GenerateBirdPdfJson(pic_info);
	GenerateBirdHtmlJson(pic_info);
}

void MainControl::GenerateBirdPdfJson(std::map<QString, std::map<QString, QString>> &pic_infos)
{
	boost::property_tree::ptree pt;
	QString out_dir = ui.label_Bird_ResDir->text();

	AddBridLogger(QStringLiteral("照片索引数量：") + QString::number(pic_infos.size()));
	if (!pic_infos.size())
		return;

	int step = 1;
	boost::property_tree::ptree errpt_array;
	QDirIterator it(out_dir + QStringLiteral("/images"), QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//遍历所有目录和文件
	AddBridLogger(QStringLiteral("读取结果图像目录：") + out_dir + QStringLiteral("/images"));
	while (it.hasNext())//存在
	{
		QApplication::processEvents();
		QString name = it.next();//读取		
		QFileInfo info(name);
		if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
		{
			AddBridLogger(QStringLiteral("可见光检测结果新增：") + info.baseName());
			boost::property_tree::ptree point_pt;
			QString quexianneirong = pic_res_[step - 1].count(QStringLiteral("鸟巢")) > 0 ? QStringLiteral("@鸟巢") : QStringLiteral("");
			quexianneirong += pic_res_[step - 1].count(QStringLiteral("绝缘子损坏")) > 0 ? QStringLiteral("@绝缘子损坏") : QStringLiteral("");
			if (pic_infos.find(info.fileName().toLower()) != pic_infos.end())
			{
				std::map<QString, QString> pic_info = pic_infos[info.fileName().toLower()];
				point_pt.put("序号", QString::number(step).toLocal8Bit().data());
				point_pt.put("线路名称", ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
				point_pt.put("杆塔编号", pic_info[QStringLiteral("塔号")].toLocal8Bit().data());
				point_pt.put("缺陷内容", quexianneirong.toLocal8Bit().data());
				point_pt.put("备注", (ui.lineEdit_Circuit_Name->text() + QStringLiteral("(") + ui.lineEdit_Circuit_Range->text() + QStringLiteral(") ") + ui.lineEdit_Kv->text()).toLocal8Bit().data());
				point_pt.put("创建日期", QDateTime::fromTime_t(pic_info[QStringLiteral("时间戳")].left(10).toInt()).toString("yyyy-MM-dd hh:mm:ss").toLocal8Bit().data());
				point_pt.put("文件名称", info.fileName().toLocal8Bit().data());
				point_pt.put("经度", pic_info[QStringLiteral("经度")].toLocal8Bit().data());
				point_pt.put("纬度", pic_info[QStringLiteral("纬度")].toLocal8Bit().data());
				point_pt.put("故障位置", pic_info[QStringLiteral("塔号")].toLocal8Bit().data());
				point_pt.put("附图", (QStringLiteral("images/") + info.fileName()).toLocal8Bit().data());
			}
			else
			{
				point_pt.put("序号", QString::number(step).toLocal8Bit().data());
				point_pt.put("线路名称", ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
				point_pt.put("杆塔编号", "");
				point_pt.put("缺陷内容", quexianneirong.toLocal8Bit().data());
				point_pt.put("备注", (ui.lineEdit_Circuit_Name->text() + QStringLiteral("(") + ui.lineEdit_Circuit_Range->text() + QStringLiteral(") ") + ui.lineEdit_Kv->text()).toLocal8Bit().data());
				point_pt.put("创建日期", "");
				point_pt.put("文件名称", info.fileName().toLocal8Bit().data());
				point_pt.put("经度", "");
				point_pt.put("纬度", "");
				point_pt.put("故障位置", "");
				point_pt.put("附图", (QStringLiteral("images/") + info.fileName()).toLocal8Bit().data());
			}
			errpt_array.push_back(std::make_pair("", point_pt));
			step++;
		}
	}

	pt.put_child("检查结果列表", errpt_array);

	AddBridLogger(QStringLiteral("正在写入：") + out_dir + QStringLiteral("/可见光检测结果.json"));
	std::string filename = (out_dir + QStringLiteral("/可见光检测结果.json")).toLocal8Bit().data();
	std::ofstream ofs(filename, std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();
	// 转成utf8
	std::string json;
	std::ifstream ifs(filename, std::ifstream::in);
	if (ifs.is_open())
	{
		std::stringstream buffer;
		buffer << ifs.rdbuf();
		json = buffer.str();
		ifs.close();
	}


	ofs.clear();
	ofs.open(filename, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		StringUtil::StringReplace(json, "\\/", "/");
		StringUtil::StringReplace(json, "\\\\", "\\");
		json = ChartSetConv::C2W(json);
		ofs << json;
		ofs.close();
	}
}

void MainControl::GenerateBirdHtmlJson(std::map<QString, std::map<QString, QString>> &pic_infos)
{
	
	QString out_dir = ui.label_Bird_ResDir->text();

	AddBridLogger(QStringLiteral("照片索引数量：") + QString::number(pic_infos.size()));
	if (!pic_infos.size())
		return;

	std::map<QString, std::vector<boost::property_tree::ptree>> pos_images;
	std::map<QString, QString> time_images;

	int step = 1;

	QDirIterator it(out_dir + QStringLiteral("/images"), QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//遍历所有目录和文件
	while (it.hasNext())//存在
	{
		QApplication::processEvents();
		QString name = it.next();//读取		
		QFileInfo info(name);
		if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
		{
			boost::property_tree::ptree image_pt;
			QString quexianneirong = pic_res_[step - 1].count(QStringLiteral("鸟巢")) > 0 ? QStringLiteral("@鸟巢") : QStringLiteral("");
			quexianneirong += pic_res_[step - 1].count(QStringLiteral("绝缘子")) > 0 ? QStringLiteral("@绝缘子") : QStringLiteral("");

			QString log = 0;
			QString lat = 0;
			QString z = 0;
			QString stime = 0;
			if (pic_infos.find(info.fileName().toLower()) != pic_infos.end())
			{
				std::map<QString, QString> pic_info = pic_infos[info.fileName().toLower()];
				image_pt.put("name", info.fileName().toLocal8Bit().data());
				image_pt.put("yaw", pic_info[QStringLiteral("航向角")].toLocal8Bit().data());
				image_pt.put("pitch", pic_info[QStringLiteral("俯仰角")].toLocal8Bit().data());
				image_pt.put("state", quexianneirong.isEmpty()? "0" : "1");

				log = pic_info[QStringLiteral("经度")];
				lat = pic_info[QStringLiteral("纬度")];
				z = pic_info[QStringLiteral("高度")];
				stime = pic_info[QStringLiteral("时间戳")];
			}
			else
			{
				image_pt.put("name", info.fileName().toLocal8Bit().data());
				image_pt.put("yaw", "");
				image_pt.put("pitch", "");
				image_pt.put("state", quexianneirong.isEmpty() ? "0" : "1");
			}

			if (log.isEmpty() || lat.isEmpty())
			{
				if (!stime.isEmpty())
					time_images[""] = stime;
				pos_images[""].push_back(image_pt);
			}
			else
			{
				std::map<QString, std::vector<boost::property_tree::ptree>>::iterator find_it = GeoUtil::GetNearImage(pos_images, log.toDouble(), lat.toDouble(), z.toDouble());
				if (find_it != pos_images.end())
				{
					find_it->second.push_back(image_pt);
					time_images[find_it->first] = stime;
				}
				else
				{
					QString key_name = log + QStringLiteral(",") + lat + QStringLiteral(",") + z;
					pos_images[key_name].push_back(image_pt);
					if (!stime.isEmpty())
						time_images[key_name] = stime;
				}
			}
			
			step++;
		}
	}

 	boost::property_tree::ptree pt;
 	boost::property_tree::ptree shoots_array;
 	for (std::map<QString, std::vector<boost::property_tree::ptree>>::iterator it = pos_images.begin();
 		it != pos_images.end(); ++it)
 	{
		QApplication::processEvents();
 		boost::property_tree::ptree shoot;
 		boost::property_tree::ptree images_array;
 
		for (std::vector<boost::property_tree::ptree>::iterator itt = it->second.begin(); itt != it->second.end(); ++itt)
		{
			images_array.push_back(std::make_pair("", *itt));
		}

 
 		shoot.put_child("images", images_array);
 		shoot.put("position", it->first.toLocal8Bit().data());
 		shoot.put("time", time_images[it->first].toLocal8Bit().data());

 		shoots_array.push_back(std::make_pair("", shoot));
 	}
 	
 
 	pt.put_child("shoots", shoots_array);

	AddBridLogger(QStringLiteral("正在写入：") + out_dir + QStringLiteral("/可见光检测结果.json"));
	std::string filename = (out_dir + QStringLiteral("/可见光检测结果.json")).toLocal8Bit().data();
	std::ofstream ofs(filename, std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();
	// 转成utf8
	std::string json;
	std::ifstream ifs(filename, std::ifstream::in);
	if (ifs.is_open())
	{
		std::stringstream buffer;
		buffer << ifs.rdbuf();
		json = buffer.str();
		ifs.close();
	}


	ofs.clear();
	ofs.open(filename, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		StringUtil::StringReplace(json, "\\/", "/");
		StringUtil::StringReplace(json, "\\\\", "\\");
		json = ChartSetConv::C2W(json);
		ofs << json;
		ofs.close();
	}
}

void MainControl::BirdRun()
{
	ui.textEdit_BridLog->clear();
	pic_res_.clear();

	QString app_dir = QApplication::applicationDirPath();
	QString pic_dir = ui.lineEdit_Bird_BirdDir->text();
	QString res_dir = ui.label_Bird_ResDir->text();

	if (!QDir().exists(pic_dir))
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("请选则照片目录。"), 0);
		return;
	}
	ui.pushButton_brid_run->setEnabled(false);
	ui.pushButton_bird_upload->setEnabled(false);

	

	statusBar()->showMessage(QFileInfo(pic_dir).baseName() + QStringLiteral("目录照片分析中..."), 0);
	
	SaveAirRouteInfo(res_dir + QStringLiteral("/线路信息.txt"));
	QApplication::processEvents();
	//QStringList args;
	//QStringList mycmd;
	//mycmd << pic_dir;
	//args << mycmd;
	//
	//
	//brid_process_->start(app_dir + QStringLiteral("/ImageRecognition/可见光检测.exe"), args);
	//brid_process_->waitForStarted();
	//
	//while (!ui.pushButton_brid_run->isEnabled())
	//{
	//	QApplication::processEvents();
	//}

	QString resimgs_dir = ui.label_Bird_ResDir->text() + QStringLiteral("/images");
	FileUtil::ReMakeDir(resimgs_dir);
	SalePictures(pic_dir, resimgs_dir);
	
	GetCounterfeitCheckInfo(resimgs_dir);
	LabelPictures(resimgs_dir);

	std::map<QString, std::map<QString, QString>> pic_info = LoadImageDescription();
	GenerateBirdPdfJson(pic_info);
	ExportBridsPdf();
	GenerateBirdHtmlJson(pic_info);

	WriteFlightPath(res_dir + QStringLiteral("/飞行路径.json"));
	statusBar()->showMessage(QFileInfo(pic_dir).baseName() + QStringLiteral("目录照片分析完成。 返回值=") + QString::number(brid_rescode_) + QStringLiteral("，是否异常退出=") + QString::number(brid_exitstatus_), 0);
	ui.pushButton_brid_run->setEnabled(true);
	ui.pushButton_bird_upload->setEnabled(true);

	ui.textEdit_BridLog->moveCursor(QTextCursor::End);
	ui.textEdit_BridLog->insertPlainText(QStringLiteral("鸟巢分析结束。\n"));
}

void MainControl::BirdOpenResultDir()
{
	QDesktopServices::openUrl(QUrl(QStringLiteral("file:") + ui.label_Bird_ResDir->text(), QUrl::TolerantMode));
}

void MainControl::ExportBridsPdf()
{
	QString app_dir = QApplication::applicationDirPath();
	AddBridLogger(QStringLiteral("生成pdf报告..."));

	QStringList args;
	QStringList mycmd;
	mycmd << QStringLiteral("--checktype") << QStringLiteral("brid") << QStringLiteral("--jsonpath") << ui.label_Bird_ResDir->text() + QStringLiteral("/可见光检测结果.json");
	args << mycmd;

	QProcess process;
	process.start(app_dir + QStringLiteral("/PdfReport/PdfReport.exe"), args);
	QApplication::processEvents();
	process.waitForFinished(15000);
	QApplication::processEvents();
}