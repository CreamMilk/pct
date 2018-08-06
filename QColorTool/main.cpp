#include "qcolortool.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    // 设置vtk警告窗口不显示
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    QColorTool w;
    w.show();
    return a.exec();
}
