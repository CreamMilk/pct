#pragma once

#include <QtWidgets/QMainWindow>
#include <QProcess>
#include "ui_MainControl.h"

class MainControl : public QMainWindow
{
	Q_OBJECT

public:
	MainControl(QWidget *parent = Q_NULLPTR);
	~MainControl();
	void SaveSetting();
	void LoadSetting();


	
	std::map<QString, std::map<QString, QString>> LoadImageDescription();
	std::map<QString, QString>  LoadFlightInfomation();

	std::vector<std::vector<QString>> LoadImageJsonDescription();
	std::vector<std::tuple<QString, QString>>  LoadFlightJsonInfomation();	


	void ExportBridsPdf();
	void SalePictures(QString in_path, QString out_path);

	void LabelPicture(QString name, QString label, int x, int y);
	void LabelPictures(QString in_path);
	void WriteFlightPath(QString filename);
	void LoadAirRouteInfo();

	void SaveAirRouteInfo(QString filename);
	void SubmitCloudWarningReport();
	void ArchiveFiles(QString name, std::vector<QString> paths);


protected:
	virtual void closeEvent(QCloseEvent *event) override;

public slots:
void GetFlightInfomationPath();

void CloudReadyReadStandardOutput();
void BridReadyReadStandardOutput();
void CloudFinished(int exitcode, QProcess::ExitStatus status);
void BridFinished(int exitcode, QProcess::ExitStatus status);


void CloudGetCloudsPath();
void CloudGetTowersDir();
void CloudGetClassDir();

void CloudRun();
void CloudOpenResultDir();

// 
 void BirdGetBirdDir();
 void BirdRun();
 void BirdOpenResultDir();

void RefreshProj();

void CloudUpLoadProj();
private:
	QProcess *cloud_process_;
	int cloud_rescode_;
	QProcess::ExitStatus cloud_exitstatus_;


	QProcess *brid_process_;
	int brid_rescode_;
	QProcess::ExitStatus brid_exitstatus_;
	std::vector<QString> pic_res_;

	Ui::MainControlClass ui;
};
