#include "MainControl.h"
#include <QtWidgets/QApplication>
#include <QStyleFactory>  

int main(int argc, char *argv[])
{
	QApplication::setStyle(QStyleFactory::create("Fusion"));
	QApplication a(argc, argv);
	MainControl w;
	w.show();
	return a.exec();
}
