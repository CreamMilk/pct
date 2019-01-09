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
	pt.put(("��������ַ�˿�"), ui.lineEdit_server_ip->text().toLocal8Bit().data());
	pt.put(("��Ŀ·��"), ui.lineEdit_projpath->text().toLocal8Bit().data());
	pt.put(("��·����"), ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
	pt.put(("����·��"), ui.lineEdit_FlightInfomation->text().toLocal8Bit().data());
	pt.put(("�ֶ�����"), ui.lineEdit_Circuit_Range->text().toLocal8Bit().data());
	pt.put(("��ѹ�ȼ�"), ui.lineEdit_Kv->text().toLocal8Bit().data());
	pt.put(("�ɼ�����"), ui.lineEdit_Date->text().toLocal8Bit().data());
	pt.put(("����.����·��"), ui.lineEdit_Cloud_CloudPath->text().toLocal8Bit().data());
	pt.put(("����.��������Ŀ¼"), ui.lineEdit_Cloud_TowerDir->text().toLocal8Bit().data());
	pt.put(("����.���Ŀ¼"), ui.label_Cloud_ResultDir->text().toLocal8Bit().data());
	pt.put(("����.����Ŀ¼"), ui.lineEdit_Cloud_ClassDir->text().toLocal8Bit().data());

	pt.put(("�ɼ���.��ƬĿ¼"), ui.lineEdit_Bird_BirdDir->text().toLocal8Bit().data());
	pt.put(("�ɼ���.���Ŀ¼"), ui.label_Bird_ResDir->text().toLocal8Bit().data());

	pt.put(("����"), ui.lineEdit_zone->text().toLocal8Bit().data());
	pt.put(("�ϰ���"), ui.radioButton_south->isChecked());
	pt.put(("������"), ui.lineEdit_batchname->text().toLocal8Bit().data());

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
		ui.lineEdit_server_ip->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��������ַ�˿�")).value().c_str()));
		ui.lineEdit_projpath->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��Ŀ·��")).value().c_str()));
		ui.lineEdit_Circuit_Name->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��·����")).value().c_str()));
		ui.lineEdit_FlightInfomation->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("����·��")).value().c_str()));
		ui.lineEdit_Circuit_Range->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("�ֶ�����")).value().c_str()));
		ui.lineEdit_Kv->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("��ѹ�ȼ�")).value().c_str()));
		ui.lineEdit_Date->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("�ɼ�����")).value().c_str()));

		boost::property_tree::ptree dianyun = pt.get_child(ChartSetConv::C2W("����"));
		ui.lineEdit_Cloud_CloudPath->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("����·��")).value().c_str()));
		ui.lineEdit_Cloud_TowerDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("��������Ŀ¼")).value().c_str()));
		ui.label_Cloud_ResultDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("���Ŀ¼")).value().c_str()));
		ui.lineEdit_Cloud_ClassDir->setText(QString::fromUtf8(dianyun.get_optional<std::string>(ChartSetConv::C2W("����Ŀ¼")).value().c_str()));

		boost::property_tree::ptree kejianguang = pt.get_child(ChartSetConv::C2W("�ɼ���"));
		ui.lineEdit_Bird_BirdDir->setText(QString::fromUtf8(kejianguang.get_optional<std::string>(ChartSetConv::C2W("��ƬĿ¼")).value().c_str()));
		ui.label_Bird_ResDir->setText(QString::fromUtf8(kejianguang.get_optional<std::string>(ChartSetConv::C2W("���Ŀ¼")).value().c_str()));


		ui.lineEdit_zone->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("����")).value().c_str()));
		ui.radioButton_south->setChecked(pt.get_optional<bool>(ChartSetConv::C2W("�ϰ���")).value());
		ui.lineEdit_batchname->setText(QString::fromUtf8(pt.get_optional<std::string>(ChartSetConv::C2W("������")).value().c_str()));
			
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
	excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
	excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //���ļ�
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt() * 2; //����

	QString Range = "A1:H" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
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
			//ui.textEdit_CloudLog->insertPlainText(heads[j] + QStringLiteral("��") + allEnvDataList_i[j] + QStringLiteral("\t"));
		}
		//ui.textEdit_CloudLog->insertPlainText(QStringLiteral("\n"));
		if (!point_info[QStringLiteral("����")].isEmpty() && !point_info[QStringLiteral("��Ƭ����")].isEmpty())
		{
			point_info[QStringLiteral("����")] = point_info[QStringLiteral("����")].replace('E', "").replace('W', '-');
			point_info[QStringLiteral("γ��")] = point_info[QStringLiteral("γ��")].replace('N', "").replace('S', '-');
			point_info[QStringLiteral("��Ƭ����")] = point_info[QStringLiteral("��Ƭ����")].toLower();
			res[point_info[QStringLiteral("��Ƭ����")]] = point_info;
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
	excel.setProperty("DisplayAlerts", false);//����ʾ�κξ�����Ϣ
	excel.setProperty("Visible", false); //���ش򿪵�excel�ļ�����
	QAxObject *workbooks = excel.querySubObject("WorkBooks");
	QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //���ļ�
	QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //���ʵ�һ��������
	QAxObject * usedrange = worksheet->querySubObject("UsedRange");
	QAxObject * rows = usedrange->querySubObject("Rows");
	int intRows = rows->property("Count").toInt() * 2; //����

	QString Range = "A1:H" + QString::number(intRows);
	QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //��ȡ��Χ
	QVariant allEnvDataQVariant = allEnvData->property("Value");
	QVariantList allEnvDataList = allEnvDataQVariant.toList();


	QStringList heads = allEnvDataList[0].toStringList();
	int log_index = heads.indexOf(QStringLiteral("����"));
	int lat_index = heads.indexOf(QStringLiteral("γ��"));
	int z_index = heads.indexOf(QStringLiteral("�߶�"));
	int time_index = heads.indexOf(QStringLiteral("ʱ���"));


	for (int i = 2; i < intRows - 2; i += 2)
	{
		QApplication::processEvents();
		QStringList allEnvDataList_i = allEnvDataList[i].toStringList();
		if (!allEnvDataList_i[time_index].isEmpty() && !allEnvDataList_i[log_index].isEmpty())
		{
			res[allEnvDataList_i[time_index]] = allEnvDataList_i[log_index].replace("E", "").replace('W', '-') + QStringLiteral(",") + allEnvDataList_i[lat_index].replace("N", "").replace('S', '-') + QStringLiteral(",") + allEnvDataList_i[z_index];
			//ui.textEdit_CloudLog->insertPlainText(allEnvDataList_i[picname_index] + QStringLiteral("��") + res[allEnvDataList_i[picname_index]] + QStringLiteral("\n"));
		}
	}

	workbook->dynamicCall("Close (Boolean)", false);
	excel.dynamicCall("Quit()");
	OleUninitialize();

	return res;
}

std::vector<std::vector<QString>> MainControl::LoadImageJsonDescription()
{
	QString imagedes = ui.label_Bird_ResDir->text() + QStringLiteral("/ͼ����Ϣ.json");
	std::vector<std::vector<QString>> res;


	boost::property_tree::ptree pt;
	// ��ȡ.json�����ļ�  utf8��ʽ��

	std::ifstream is;
	is.open(imagedes.toLocal8Bit().data(), std::ios::in);
	if (is.is_open())
	{
		try {
			read_json(is, pt);          //parse json
			is.close();
		}
		catch (...) {
			QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("����·��.json��ȡʧ�ܣ�"), 0);
			return res;
		}
		is.close();
	}
	else
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("ͼ����Ϣ��ʧ�ܣ�"), 0);
		return res;
	}

	try
	{
		BOOST_FOREACH(boost::property_tree::ptree::value_type &cvt, pt)
		{
			std::vector<QString> img;
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("name")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("ƫ����")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("������")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("����")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("γ��")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("�߶�")).c_str()));
			img.push_back(QString::fromUtf8(cvt.second.get<std::string>(ChartSetConv::C2W("ʱ��")).c_str()));
			res.push_back(img);
		}
	}
	catch (...)
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("ͼ����Ϣ��ȡʧ�ܣ�"), 0);
		return res;
	}


	return res;
}

std::vector<std::tuple<QString, QString>> MainControl::LoadFlightJsonInfomation()
{
	QString filename = ui.lineEdit_FlightInfomation->text();
	std::vector<std::tuple<QString, QString>> res;


	boost::property_tree::ptree pt;
	// ��ȡ.json�����ļ�  utf8��ʽ��

	std::ifstream is;
	is.open(filename.toLocal8Bit().data(), std::ios::in);
	if (is.is_open())
	{
		try {
			read_json(is, pt);          //parse json
			is.close();
		}
		catch (...) {
			QMessageBox::information(this, "��ҵ��ʾ", "����·����ȡʧ�ܣ�", 0);
			return res;
		}
		is.close();
	}
	else
	{
		QMessageBox::information(this, "��ҵ��ʾ", "����·����ʧ�ܣ�", 0);
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
		QMessageBox::information(this, "��ҵ��ʾ", "����·����ȡʧ�ܣ�", 0);
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
	QString projcode = ServerFunc::GetProjectCode();
	QString serverport = ServerFunc::GetServerPort();
	if (projcode.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("���¼��Ǻ��л���ǰ��Ŀ��"));
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


	// ׼������
	QString resdir = ui.label_Cloud_ResultDir->text();
	QString flightpath_path = resdir + QStringLiteral("/����·��.json");
	std::vector<QString> paths;
	paths.push_back(resdir + QStringLiteral("/���Ƽ����.pdf"));
	paths.push_back(resdir + QStringLiteral("/���Ƽ����.json"));
	if (QFile::exists(flightpath_path))
		paths.push_back(flightpath_path);

	QString zip_file = ui.label_Cloud_ResultDir->text() + QStringLiteral("/") + batchname + QStringLiteral(".zip");
	ArchiveFiles(zip_file, paths);

	// ����ύ����
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


/*    ���Դ���
QString dir = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
std::vector<QString> paths;
paths.push_back(dir + QStringLiteral("/�½��ı��ĵ�.txt"));
paths.push_back(dir + QStringLiteral("/��·��Ϣ.txt"));
paths.push_back(dir + QStringLiteral("/data-20181220-042719�����.pdf"));
paths.push_back(dir + QStringLiteral("/gui-config.json"));
setWindowTitle(dir);
ArchiveFiles(dir + QStringLiteral("/��a.zip"), paths);
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

	// ׼������
	QString resdir = ui.label_Bird_ResDir->text();
	QString flightpath_path = resdir + QStringLiteral("/����·��.json");
	std::vector<QString> paths;
	paths.push_back(resdir + QStringLiteral("/�ɼ�������.pdf"));
	paths.push_back(resdir + QStringLiteral("/�ɼ�������.json"));
	paths.push_back(resdir + QStringLiteral("/images"));
	if (QFile::exists(flightpath_path))
		paths.push_back(flightpath_path);

	QString zip_file = output_dir + QStringLiteral("/") + batchname + QStringLiteral(".zip");
	ArchiveFiles(zip_file, paths);

	// ����ύ����
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
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("����������Ϊ�ա�"));
		return;
	}
	if (!QFile::exists(output_dir + QStringLiteral("/�ɼ�������.pdf")))
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("û�п��ϴ��ı��档"));
		return;
	}

	SubmitBridWarningReport();
	statusBar()->showMessage(QFileInfo(output_dir).baseName() + QStringLiteral("��������ϴ��ɹ���"), 0);
}

void MainControl::CloudUpLoadProj()
{
	QString projpath = ui.lineEdit_projpath->text();
	QString projdir = projpath + QStringLiteral("/") + ServerFunc::GetProjectCode() +  QStringLiteral("/mds");
	QString output_dir = ui.label_Cloud_ResultDir->text();
	QString batch_name = ui.lineEdit_batchname->text();

	if (batch_name.isEmpty())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("����������Ϊ�ա�"));
		return;
	}
	if (!QDir(projpath).exists())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("��Ŀ·�������ڻ���û�з���Ȩ�ޡ�"));
		return;
	}
	if (!QDir(output_dir + QStringLiteral("/ground")).exists())
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("���ȷ������ơ�"));
		return;
	}
	if (!QFile::exists(output_dir + QStringLiteral("/���Ƽ����.pdf")))
	{
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("û�п��ϴ��ı��档"));
		return;
	}

	if (!QDir(projdir).exists())
	{
		QDir().mkpath(projdir);
		if (!QDir(projdir).exists())
		{
			QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("��ĿĿ¼����ʧ�ܣ����ֶ�������"));
			return;
		}
	}

	FileUtil::CopyDirectory(output_dir + QStringLiteral("/ground"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/ground"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/lines"),  projdir + QStringLiteral("/") + batch_name + QStringLiteral("/lines"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/others"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/others"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/towers"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/towers"));
	FileUtil::CopyDirectory(output_dir + QStringLiteral("/vegets"), projdir + QStringLiteral("/") + batch_name + QStringLiteral("/vegets"));

	SubmitCloudWarningReport();
	statusBar()->showMessage(QFileInfo(output_dir).baseName() + QStringLiteral("��������ϴ��ɹ���"), 0);
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

	FileUtil::ReMakeDir(ui.label_Cloud_ResultDir->text());
	SaveAirRouteInfo(ui.label_Cloud_ResultDir->text() + QStringLiteral("/��·��Ϣ.txt"));


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


	WriteFlightPath(ui.label_Cloud_ResultDir->text() + QStringLiteral("/����·��.json"));

	statusBar()->showMessage(QFileInfo(las_path).baseName() + QStringLiteral(".las������ɡ� ����ֵ=") + QString::number(cloud_rescode_) + QStringLiteral("���Ƿ��쳣�˳�=") + QString::number(cloud_exitstatus_), 0);
}

void MainControl::GetFlightInfomationPath()
{
	QString path = QFileDialog::getOpenFileName(this, QStringLiteral("��ѡ����·���ļ�..."), QFileInfo(ui.lineEdit_FlightInfomation->text()).absoluteDir().path(), QStringLiteral("Flight Infomation(*.xlsx *.xls)"));
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

	if (0 == cmd_print.indexOf(QStringLiteral("ʶ������")))
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
	QString path = QFileDialog::getExistingDirectory(this, QStringLiteral("��ѡ����ƬĿ¼..."), ui.lineEdit_Bird_BirdDir->text());
	if (path.length() == 0) {
		return;
	}

	ui.lineEdit_Bird_BirdDir->setText(path);
	ui.label_Bird_ResDir->setText(path);
}

void MainControl::SalePictures(QString in_path, QString out_path)
{
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//��������Ŀ¼���ļ�

	while (it.hasNext())//����
	{
		QString name = it.next();//��ȡ		
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
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//��������Ŀ¼���ļ�
	QTextCodec *codec(QTextCodec::codecForName("UTF-8"));

	while (it.hasNext())//����
	{
		QString name = it.next();//��ȡ		
		QFileInfo info(name);
		if (!info.isDir())
		{

			if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
			{
				std::chrono::system_clock::time_point start = std::chrono::system_clock::now();


				QFile boxsinfo(QStringLiteral(":/jpgpymd5boxs.txt"));

				ui.textEdit_BridLog->moveCursor(QTextCursor::End);
				ui.textEdit_BridLog->insertPlainText(QStringLiteral("������Ƭ��") + name +  QStringLiteral("\n"));
				QString shibiejieguo = QStringLiteral("ʶ������");
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
									obj_name = QStringLiteral("��");
								}
								else if (sl[obj_base_index + 4].trimmed() == QStringLiteral("2"))
								{
									obj_name = QStringLiteral("��Ե��");
								}
								else if (sl[obj_base_index + 4].trimmed() == QStringLiteral("3"))
								{
									obj_name = QStringLiteral("��Ե����");
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
					ui.textEdit_BridLog->insertPlainText(QStringLiteral("ʶ������\n"));
				}
				else
				{
					pic_res_.push_back(shibiejieguo_val);
					ui.textEdit_BridLog->moveCursor(QTextCursor::End);
					ui.textEdit_BridLog->insertPlainText(QStringLiteral("ʶ������") + shibiejieguo_val + QStringLiteral("\n"));
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
	QPainter painter;//ע�ⲻҪ����(this)��thisָ��ֱ����mainwindow��ͼ

	QImage image(name);//����ͼƬ������ͼƬ�ϻ�ͼ������ʾ

	painter.begin(&image);

	QPen pen;
	pen.setWidth(3);
	pen.setColor(QColor(rgb  << 8 >> 24, rgb << 16 >> 24, rgb << 24 >> 24));
	//pen.setColor(Qt::red);
	painter.setPen(pen);
	QFont font;
	font.setPointSize(18);
	painter.setFont(font);

	painter.drawRect(x, y, maxx, maxy);//����
	painter.drawText(x, y - 15, label);
	painter.end();

	image.save(name);
}

void MainControl::LabelPictures(QString in_path)
{
	QDirIterator it(in_path, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//��������Ŀ¼���ļ�

	int step = 0;
	while (it.hasNext() && step < pic_res_.size())
	{
		QString name = it.next();//��ȡ		
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
					if (0 == single.indexOf(QStringLiteral("��@")))
					{
						QStringList xy = single.right(single.length() - 3).split(',');
						LabelPicture(name, QStringLiteral("��"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt());
					}
					else if(0 == single.indexOf(QStringLiteral("��Ե��@")))
					{
						QStringList xy = single.right(single.length() - 4).split(',');
						LabelPicture(name, QStringLiteral("��Ե��"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt(), 0x0000FF00);
					}
					else if (0 == single.indexOf(QStringLiteral("��Ե����@")))
					{
						QStringList xy = single.right(single.length() - 4).split(',');
						LabelPicture(name, QStringLiteral("��Ե����"), xy[0].toInt(), xy[1].toInt(), xy[2].toInt(), xy[3].toInt());
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

	AddBridLogger(QStringLiteral("��Ƭ����������") + QString::number(pic_infos.size()));
	if (!pic_infos.size())
		return;

	int step = 1;
	boost::property_tree::ptree errpt_array;
	QDirIterator it(out_dir + QStringLiteral("/images"), QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//��������Ŀ¼���ļ�
	AddBridLogger(QStringLiteral("��ȡ���ͼ��Ŀ¼��") + out_dir + QStringLiteral("/images"));
	while (it.hasNext())//����
	{
		QApplication::processEvents();
		QString name = it.next();//��ȡ		
		QFileInfo info(name);
		if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
		{
			AddBridLogger(QStringLiteral("�ɼ�������������") + info.baseName());
			boost::property_tree::ptree point_pt;
			QString quexianneirong = pic_res_[step - 1].count(QStringLiteral("��")) > 0 ? QStringLiteral("@��") : QStringLiteral("");
			quexianneirong += pic_res_[step - 1].count(QStringLiteral("��Ե����")) > 0 ? QStringLiteral("@��Ե����") : QStringLiteral("");
			if (pic_infos.find(info.fileName().toLower()) != pic_infos.end())
			{
				std::map<QString, QString> pic_info = pic_infos[info.fileName().toLower()];
				point_pt.put("���", QString::number(step).toLocal8Bit().data());
				point_pt.put("��·����", ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
				point_pt.put("�������", pic_info[QStringLiteral("����")].toLocal8Bit().data());
				point_pt.put("ȱ������", quexianneirong.toLocal8Bit().data());
				point_pt.put("��ע", (ui.lineEdit_Circuit_Name->text() + QStringLiteral("(") + ui.lineEdit_Circuit_Range->text() + QStringLiteral(") ") + ui.lineEdit_Kv->text()).toLocal8Bit().data());
				point_pt.put("��������", QDateTime::fromTime_t(pic_info[QStringLiteral("ʱ���")].left(10).toInt()).toString("yyyy-MM-dd hh:mm:ss").toLocal8Bit().data());
				point_pt.put("�ļ�����", info.fileName().toLocal8Bit().data());
				point_pt.put("����", pic_info[QStringLiteral("����")].toLocal8Bit().data());
				point_pt.put("γ��", pic_info[QStringLiteral("γ��")].toLocal8Bit().data());
				point_pt.put("����λ��", pic_info[QStringLiteral("����")].toLocal8Bit().data());
				point_pt.put("��ͼ", (QStringLiteral("images/") + info.fileName()).toLocal8Bit().data());
			}
			else
			{
				point_pt.put("���", QString::number(step).toLocal8Bit().data());
				point_pt.put("��·����", ui.lineEdit_Circuit_Name->text().toLocal8Bit().data());
				point_pt.put("�������", "");
				point_pt.put("ȱ������", quexianneirong.toLocal8Bit().data());
				point_pt.put("��ע", (ui.lineEdit_Circuit_Name->text() + QStringLiteral("(") + ui.lineEdit_Circuit_Range->text() + QStringLiteral(") ") + ui.lineEdit_Kv->text()).toLocal8Bit().data());
				point_pt.put("��������", "");
				point_pt.put("�ļ�����", info.fileName().toLocal8Bit().data());
				point_pt.put("����", "");
				point_pt.put("γ��", "");
				point_pt.put("����λ��", "");
				point_pt.put("��ͼ", (QStringLiteral("images/") + info.fileName()).toLocal8Bit().data());
			}
			errpt_array.push_back(std::make_pair("", point_pt));
			step++;
		}
	}

	pt.put_child("������б�", errpt_array);

	AddBridLogger(QStringLiteral("����д�룺") + out_dir + QStringLiteral("/�ɼ�������.json"));
	std::string filename = (out_dir + QStringLiteral("/�ɼ�������.json")).toLocal8Bit().data();
	std::ofstream ofs(filename, std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();
	// ת��utf8
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

	AddBridLogger(QStringLiteral("��Ƭ����������") + QString::number(pic_infos.size()));
	if (!pic_infos.size())
		return;

	std::map<QString, std::vector<boost::property_tree::ptree>> pos_images;
	std::map<QString, QString> time_images;

	int step = 1;

	QDirIterator it(out_dir + QStringLiteral("/images"), QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);	//��������Ŀ¼���ļ�
	while (it.hasNext())//����
	{
		QApplication::processEvents();
		QString name = it.next();//��ȡ		
		QFileInfo info(name);
		if (info.suffix().toLower() == "jpg" || info.suffix().toLower() == "bmp" || info.suffix().toLower() == "png" || info.suffix().toLower() == "jpeg")
		{
			boost::property_tree::ptree image_pt;
			QString quexianneirong = pic_res_[step - 1].count(QStringLiteral("��")) > 0 ? QStringLiteral("@��") : QStringLiteral("");
			quexianneirong += pic_res_[step - 1].count(QStringLiteral("��Ե��")) > 0 ? QStringLiteral("@��Ե��") : QStringLiteral("");

			QString log = 0;
			QString lat = 0;
			QString z = 0;
			QString stime = 0;
			if (pic_infos.find(info.fileName().toLower()) != pic_infos.end())
			{
				std::map<QString, QString> pic_info = pic_infos[info.fileName().toLower()];
				image_pt.put("name", info.fileName().toLocal8Bit().data());
				image_pt.put("yaw", pic_info[QStringLiteral("�����")].toLocal8Bit().data());
				image_pt.put("pitch", pic_info[QStringLiteral("������")].toLocal8Bit().data());
				image_pt.put("state", quexianneirong.isEmpty()? "0" : "1");

				log = pic_info[QStringLiteral("����")];
				lat = pic_info[QStringLiteral("γ��")];
				z = pic_info[QStringLiteral("�߶�")];
				stime = pic_info[QStringLiteral("ʱ���")];
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

	AddBridLogger(QStringLiteral("����д�룺") + out_dir + QStringLiteral("/�ɼ�������.json"));
	std::string filename = (out_dir + QStringLiteral("/�ɼ�������.json")).toLocal8Bit().data();
	std::ofstream ofs(filename, std::fstream::out);
	boost::property_tree::write_json(ofs, pt);
	ofs.close();
	// ת��utf8
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
		QMessageBox::information(this, QStringLiteral("��ҵ��ʾ"), QStringLiteral("��ѡ����ƬĿ¼��"), 0);
		return;
	}
	ui.pushButton_brid_run->setEnabled(false);
	ui.pushButton_bird_upload->setEnabled(false);

	

	statusBar()->showMessage(QFileInfo(pic_dir).baseName() + QStringLiteral("Ŀ¼��Ƭ������..."), 0);
	
	SaveAirRouteInfo(res_dir + QStringLiteral("/��·��Ϣ.txt"));
	QApplication::processEvents();
	//QStringList args;
	//QStringList mycmd;
	//mycmd << pic_dir;
	//args << mycmd;
	//
	//
	//brid_process_->start(app_dir + QStringLiteral("/ImageRecognition/�ɼ�����.exe"), args);
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

	WriteFlightPath(res_dir + QStringLiteral("/����·��.json"));
	statusBar()->showMessage(QFileInfo(pic_dir).baseName() + QStringLiteral("Ŀ¼��Ƭ������ɡ� ����ֵ=") + QString::number(brid_rescode_) + QStringLiteral("���Ƿ��쳣�˳�=") + QString::number(brid_exitstatus_), 0);
	ui.pushButton_brid_run->setEnabled(true);
	ui.pushButton_bird_upload->setEnabled(true);

	ui.textEdit_BridLog->moveCursor(QTextCursor::End);
	ui.textEdit_BridLog->insertPlainText(QStringLiteral("�񳲷���������\n"));
}

void MainControl::BirdOpenResultDir()
{
	QDesktopServices::openUrl(QUrl(QStringLiteral("file:") + ui.label_Bird_ResDir->text(), QUrl::TolerantMode));
}

void MainControl::ExportBridsPdf()
{
	QString app_dir = QApplication::applicationDirPath();
	AddBridLogger(QStringLiteral("����pdf����..."));

	QStringList args;
	QStringList mycmd;
	mycmd << QStringLiteral("--checktype") << QStringLiteral("brid") << QStringLiteral("--jsonpath") << ui.label_Bird_ResDir->text() + QStringLiteral("/�ɼ�������.json");
	args << mycmd;

	QProcess process;
	process.start(app_dir + QStringLiteral("/PdfReport/PdfReport.exe"), args);
	QApplication::processEvents();
	process.waitForFinished(15000);
	QApplication::processEvents();
}