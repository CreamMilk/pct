#include "qcolortool.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    // ����vtk���洰�ڲ���ʾ
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    QColorTool w;
    w.show();
    return a.exec();
}
