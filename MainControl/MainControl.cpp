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
	pt.put(("��·����"), ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
	pt.put(("�ֶ�����"), ui.lineEdit_Circuit_Range->text().toLocal8Bit().data());
	pt.put(("��ѹ�ȼ�"), ui.lineEdit_Kv->text().toLocal8Bit().data());
	pt.put(("�ɼ�����"), ui.lineEdit_Date->text().toLocal8Bit().data());
	pt.put(("����.����·��"), ui.lineEdit_Cloud_CloudPath->text().toLocal8Bit().data());
	pt.put(("����.��������Ŀ¼"), ui.lineEdit_Cloud_TowerDir->text().toLocal8Bit().data());
	pt.put(("����.���Ŀ¼"), ui.label_Cloud_ResultDir->text().toLocal8Bit().data());
	pt.put(("����.����Ŀ¼"), ui.lineEdit_Cloud_ClassDir->text().toLocal8Bit().data());

	std::ofstream ofs(filename.toLocal8Bit().data(), std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();


	// ת��utf8
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
		ui.lineEdit_Circuit_Name->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��·����")).value().c_str()));
		ui.lineEdit_Circuit_Range->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("�ֶ�����")).value().c_str()));
		ui.lineEdit_Kv->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��ѹ�ȼ�")).value().c_str()));
		ui.lineEdit_Date->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("�ɼ�����")).value().c_str()));

		boost::property_tree::ptree dianyun = pt.get_child(ChartSetConv::C2W("����"));
		ui.lineEdit_Cloud_CloudPath->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("����·��")).value().c_str()));
		ui.lineEdit_Cloud_TowerDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("��������Ŀ¼")).value().c_str()));
		ui.label_Cloud_ResultDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("���Ŀ¼")).value().c_str()));
		ui.lineEdit_Cloud_ClassDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("����Ŀ¼")).value().c_str()));
	}
	catch (...) {
		return;
	}
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
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("��ѡ������ļ�..."), QFileInfo(ui.lineEdit_Cloud_CloudPath->text()).absoluteDir().path(), QStringLiteral("Clouds File(*.las)"));
	if (path.length() == 0) {
		return;
	}

	QString res_dir = QFileInfo(path).absoluteDir().path() + QStringLiteral("/") + QFileInfo(path).baseName();
	ui.lineEdit_Cloud_CloudPath->setText(path);
	ui.label_Cloud_ResultDir->setText(res_dir);
}

void MainControl::CloudGetTowersDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("��ѡ������Ŀ¼..."), ui.lineEdit_Cloud_TowerDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_TowerDir->setText(path);
}

void MainControl::CloudGetClassDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("��ѡ������Ŀ¼..."), ui.lineEdit_Cloud_ClassDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_ClassDir->setText(path);
}

void MainControl::LoadAirRouteInfo()
{
	QString filename = ui.label_Cloud_ResultDir->text() + QStringLiteral("/��·��Ϣ.txt");

	QFile file(filename);
	// Trying to open in WriteOnly and Text mode
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		return;
	}

	QString Circuit_Name, Circuit_Range, Kv, Date;
	QTextStream out(&file);
	out >> /*QStringLiteral("��·����=") >>*/ Circuit_Name;
	out >> /*QStringLiteral("�ֶ�����=") >>*/ Circuit_Range ;
	out >> /*QStringLiteral("��ѹ�ȼ�=") >>*/ Kv ;
	out >> /*QStringLiteral("�ɼ�����=") >>*/ Date ;

	Circuit_Name = Circuit_Name.right(Circuit_Name.length() - Circuit_Name.indexOf(QStringLiteral("=")) -1);
	Circuit_Range = Circuit_Range.right(Circuit_Range.length() - Circuit_Range.indexOf(QStringLiteral("=")) - 1);
	Kv = Kv.right(Kv.length() - Kv.indexOf(QStringLiteral("="))- 1);
	Date = Date.right(Date.length() - Date.indexOf(QStringLiteral("=")) - 1);

	file.close();
}

void MainControl::SaveAirRouteInfo()
{
	QString filename = ui.label_Cloud_ResultDir->text() + QStringLiteral("/��·��Ϣ.txt");

	QFile file(filename);
	// Trying to open in WriteOnly and Text mode
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		return;
	}

	QTextStream out(&file);
	out << QStringLiteral("��·����=") << ui.lineEdit_Circuit_Name->text() << QStringLiteral("\n");
	out << QStringLiteral("�ֶ�����=") << ui.lineEdit_Circuit_Range->text() << QStringLiteral("\n");
	out << QStringLiteral("��ѹ�ȼ�=") << ui.lineEdit_Kv->text() << QStringLiteral("\n");
	out << QStringLiteral("�ɼ�����=") << ui.lineEdit_Date->text() << QStringLiteral("\n");
	file.flush();
	file.close();
}

void MainControl::RefreshProj()
{
	QString projname = ServerFunc::GetProjectName();
	if (projname.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("���¼��̺��л���ǰ��Ŀ��"));
	}
	ui.lineEdit_projname->setText(projname);
}

void MainControl::UpLoadProj()
{
	QString output_dir = ui.lineEdit_Cloud_ClassDir->text();
	if (/*�����Ƿ�߱��ϴ�����*/1)
		;

	if (!QDir(QStringLiteral("//192.168.6.27")).exists())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("Ŀ¼�����ڻ���û�з���Ȩ�ޡ�"));
	}

}

void MainControl::CloudRun()
{
	ui.textEdit_CloudLog->clear();
	
	QStringList args;

	QString app_dir = QApplication::applicationDirPath();
	QString las_path = ui.lineEdit_Cloud_CloudPath->text();
	if (!QFile::exists(las_path))
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("����·�������ڡ�"), 0);
		return;
	}
	QString class_path = ui.lineEdit_Cloud_ClassDir->text() + QStringLiteral("/config.xml");
	if (!QFile::exists(class_path))
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("����Ŀ¼�в�����ѵ���ļ�(config.xml)��"), 0);
		return;
	}
	QString tower_dir = ui.lineEdit_Cloud_TowerDir->text();
	if (!QDir().exists(tower_dir))
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("��ѡ������Ŀ¼��"), 0);
		return;
	}

	
	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las������..."), 0);


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

	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las������ɡ� ����ֵ=") + QString::number(cloud_rescode_) + QStringLiteral("���Ƿ��쳣�˳�=") + QString::number(cloud_exitstatus_), 0);
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
