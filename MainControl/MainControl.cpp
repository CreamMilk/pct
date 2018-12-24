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
#include "CommonFuns.h"
#include "ServerFunc.h"

MainControl::MainControl(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
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
	pt.put(("线路名称"), ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
	pt.put(("航行路径"), ui.lineEdit_FlightInfomation->text().toLocal8Bit().data());
	pt.put(("分段区间"), ui.lineEdit_Circuit_Range->text().toLocal8Bit().data());
	pt.put(("电压等级"), ui.lineEdit_Kv->text().toLocal8Bit().data());
	pt.put(("采集日期"), ui.lineEdit_Date->text().toLocal8Bit().data());
	pt.put(("点云.点云路径"), ui.lineEdit_Cloud_CloudPath->text().toLocal8Bit().data());
	pt.put(("点云.铁塔数据目录"), ui.lineEdit_Cloud_TowerDir->text().toLocal8Bit().data());
	pt.put(("点云.结果目录"), ui.label_Cloud_ResultDir->text().toLocal8Bit().data());
	pt.put(("点云.样本目录"), ui.lineEdit_Cloud_ClassDir->text().toLocal8Bit().data());

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
	}
	catch (...) {
		return;
	}
}

std::vector<std::vector<QString>> MainControl::LoadImageDescription()
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

std::vector<std::tuple<QString, QString>> MainControl::LoadFlightInfomation()
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

void MainControl::SaveAirRouteInfo()
{
	QString filename = ui.label_Cloud_ResultDir->text() + QStringLiteral("/线路信息.txt");

	QFile file(filename);
	// Trying to open in WriteOnly and Text mode
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
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
	if (projcode.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("鸿业提示"), QStringLiteral("请登录鸿程后切换当前项目。"));
		return;
	}
	ui.label_projname->setText(projname);
	ui.label_projcode->setText(projcode);
}

void MainControl::SubmitCloudWarningReport()
{
	QString output_dir = ui.label_Cloud_ResultDir->text();
	//QTextCodec* codec = QTextCodec::codecForName("UTF-8");

	std::shared_ptr<QNetworkAccessManager> proc_manager = std::make_shared<QNetworkAccessManager>();
	std::shared_ptr<QHttpMultiPart> multiPart = std::make_shared<QHttpMultiPart>(QHttpMultiPart::FormDataType);
	
	
	QHttpPart ProjectCode;
	ProjectCode.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"ProjectCode\"")));
	ProjectCode.setBody(ServerFunc::GetProjectCode().toUtf8());

	QHttpPart pdfFile;
	pdfFile.setHeader(QNetworkRequest::ContentTypeHeader, QVariant(QStringLiteral("application/pdf")));
	pdfFile.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"File\"")));
	QFile file(output_dir + QStringLiteral("/检测结果.pdf"));
	file.open(QIODevice::ReadOnly);
	pdfFile.setBodyDevice(&file);

	QHttpPart FlyCode;
	FlyCode.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"FlyCode\"")));
	FlyCode.setBody(QDir(output_dir).dirName().toUtf8());

	QHttpPart DisplyFileName;
	DisplyFileName.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant(QStringLiteral("form-data; name=\"DisplyFileName\"")));
	DisplyFileName.setBody(QDir(output_dir).dirName().toUtf8());


	multiPart->append(ProjectCode);
	multiPart->append(pdfFile);
	multiPart->append(FlyCode);
	multiPart->append(DisplyFileName);


	QNetworkReply *reply = proc_manager->post(QNetworkRequest(QUrl(ui.lineEdit_server_ip->text())), multiPart.get());
	QByteArray responseData;
	QEventLoop eventLoop;
	connect(proc_manager.get(), SIGNAL(finished(QNetworkReply*)), &eventLoop, SLOT(quit()));
	eventLoop.exec();
	responseData = reply->readAll();
	QString response_str = QString::fromUtf8(responseData.data());
	file.close();
}

void MainControl::CloudUpLoadProj()
{
	QString projpath = ui.lineEdit_projpath->text();
	QString projdir = projpath + QStringLiteral("/") + ServerFunc::GetProjectCode() +  QStringLiteral("/mds");
	QString output_dir = ui.label_Cloud_ResultDir->text();

	
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
	if (!QFile::exists(output_dir + QStringLiteral("/检测结果.pdf")))
	{
		
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

	FileUtil::CopyDirectory(output_dir + QStringLiteral("/ground"), projdir + QStringLiteral("/ground"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/lines"), projdir + QStringLiteral("/lines"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/others"), projdir + QStringLiteral("/others"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/towers"), projdir + QStringLiteral("/towers"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/vegets"), projdir + QStringLiteral("/vegets"));
	QFile::copy(output_dir + QStringLiteral("/检测结果.json"), projdir + QStringLiteral("/检测结果.json"));

	SubmitCloudWarningReport();
	
}

void MainControl::CloudRun()
{
	LoadImageDescription();
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
	SaveAirRouteInfo();


	QStringList mycmd;
	mycmd << QStringLiteral("--cmdtype") << QStringLiteral("poscorrect") << QStringLiteral("--inputfile") << las_path << QStringLiteral("--classdir") << ui.lineEdit_Cloud_ClassDir->text() <<
		QStringLiteral("--method") << QString::number(2) << QStringLiteral("--exceldir") << tower_dir << QStringLiteral("--overrideExcel") << QStringLiteral("0");



	args << mycmd;
	ui.pushButton_cloud_run->setEnabled(false);

	cloud_process_->start(app_dir + QStringLiteral("/PointCloudTool.exe"), args);
	cloud_process_->waitForStarted();

	while (!ui.pushButton_cloud_run->isEnabled())
	{
		QApplication::processEvents();
	}

	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las分析完成。 返回值=") + QString::number(cloud_rescode_) + QStringLiteral("，是否异常退出=") + QString::number(cloud_exitstatus_), 0);
}

void MainControl::GetFlightInfomationPath()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("请选择航行路径文件..."), QFileInfo(ui.lineEdit_FlightInfomation->text()).absoluteDir().path(), QStringLiteral("Flight Infomation(*.json)"));
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_FlightInfomation->setText(path);
}

void MainControl::CloudReadyReadStandardOutput()
{
	ui.textEdit_CloudLog->insertPlainText(QTextCodec::codecForName("GB2312")->toUnicode(cloud_process_->readAll()));
	ui.textEdit_CloudLog->moveCursor(QTextCursor::End);
}

void MainControl::BridReadyReadStandardOutput()
{
	ui.textEdit_BridLog->insertPlainText(QTextCodec::codecForName("GB2312")->toUnicode(brid_process_->readAll()));
	ui.textEdit_BridLog->moveCursor(QTextCursor::End);
}

void MainControl::CloudFinished(int exitcode, QProcess::ExitStatus status)
{
	cloud_rescode_ = exitcode;
	cloud_exitstatus_ = status;
	ui.pushButton_cloud_run->setEnabled(true);
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

void MainControl::BirdRun()
{
	ui.textEdit_BridLog->clear();
}
