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
#include "CommonFuns.h"
#include <QProcess>

MainControl::MainControl(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	LoadSetting();
}


void MainControl::SaveSetting()
{
 	QString filename = QApplication::applicationDirPath() + QStringLiteral("/GuiConfig.json");
	boost::property_tree::ptree pt;
	pt.put(("线路名称"), ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
	pt.put(("分段区间"), ui.lineEdit_Circuit_Range->text().toLocal8Bit().data());
	pt.put(("电压等级"), ui.lineEdit_Kv->text().toLocal8Bit().data());
	pt.put(("采集日期"), ui.lineEdit_Date->text().toLocal8Bit().data());
	pt.put(("点云.点云目录"), ui.lineEdit_Cloud_CloudDir->text().toLocal8Bit().data());
	pt.put(("点云.铁塔数据目录"), ui.lineEdit_Cloud_TowerDir->text().toLocal8Bit().data());
	pt.put(("点云.结果目录"), ui.label_Cloud_ResultDir->text().toLocal8Bit().data());

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
	
	ui.lineEdit_Circuit_Name->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("线路名称")).value().c_str()));
	ui.lineEdit_Circuit_Range->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("分段区间")).value().c_str()));
	ui.lineEdit_Kv->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("电压等级")).value().c_str()));
	ui.lineEdit_Date->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("采集日期")).value().c_str()));

	boost::property_tree::ptree dianyun = pt.get_child(ChartSetConv::C2W("点云"));
	ui.lineEdit_Cloud_CloudDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("点云目录")).value().c_str()));
	ui.lineEdit_Cloud_TowerDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("铁塔数据目录")).value().c_str()));
	ui.label_Cloud_ResultDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("结果目录")).value().c_str()));
}

void MainControl::closeEvent(QCloseEvent *event)
{
	SaveSetting();
	QMainWindow::closeEvent(event);
}

void MainControl::CloudGetCloudsDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("请选择点云目录..."), QStringLiteral("./"));
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_CloudDir->setText(path);
	ui.label_Cloud_ResultDir->setText(path);
}

void MainControl::CloudGetTowersDir()
{
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("请选择铁塔目录..."), QStringLiteral("./"));
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Cloud_TowerDir->setText(QFileInfo(path).absoluteDir().absolutePath());
}

void MainControl::CloudRun()
{
	QProcess _process;

	connect(&_process, &QProcess::readyReadStandardOutput, this, &MainControl::onReadOutput);
	connect(&_process, &QProcess::readyReadStandardError, this, &MainControl::onReadOutput);
	//connect(&_process, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(onFinished(int, QProcess::ExitStatus)));

	QString cmd = QString("ping localhost");
	_process.start(cmd);
}

void MainControl::onReadOutput()
{
	static int i = 0;
	i++;
	setWindowTitle(QString::number(i));
}

void MainControl::CloudOpenResultDir()
{
	QDesktopServices::openUrl(QUrl(QStringLiteral("file:") + ui.label_Cloud_ResultDir->text(), QUrl::TolerantMode));
}
