#include "MainControl.h"
#include <QtWidgets/QApplication>
#include <QStyleFactory>  
#include <QFile>

int main(int argc, char *argv[])
{
	QApplication::setStyle(QStyleFactory::create("Fusion"));
	QApplication a(argc, argv);
	MainControl w;

	QFile qss(QStringLiteral(":/MainControl.qss"));
	qss.open(QFile::ReadOnly);
	w.setStyleSheet(qss.readAll());
	qss.close();

	w.show();
	return a.exec();
}
