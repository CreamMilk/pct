#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainControl.h"

class MainControl : public QMainWindow
{
	Q_OBJECT

public:
	MainControl(QWidget *parent = Q_NULLPTR);
	
public slots:
void CloudGetCloudsDir();
void CloudGetTowersDir();
void CloudRun();
void CloudOpenResultDir();
void UpLoad();

void BirdGetBirdDir();
void BirdRun();
void BirdOpenResultDir();

private:
	Ui::MainControlClass ui;
};
