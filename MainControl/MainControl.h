#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainControl.h"

class MainControl : public QMainWindow
{
	Q_OBJECT

public:
	MainControl(QWidget *parent = Q_NULLPTR);

private:
	Ui::MainControlClass ui;
};
