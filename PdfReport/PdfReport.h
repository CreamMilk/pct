#pragma once

#include <QDialog>
#include "ui_PdfReport.h"

class PdfReport : public QDialog
{
	Q_OBJECT

public:
	PdfReport(QWidget *parent = Q_NULLPTR);

private:
	Ui::PdfReportClass ui;
};
