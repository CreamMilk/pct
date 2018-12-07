#include "MainControl.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainControl w;
	w.show();
	return a.exec();
}
