#include <iostream>
#include <string>
#include <io.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/typeof/typeof.hpp>   
#include <boost/locale.hpp>
#include <chrono>   
#include <Scene_points_with_normal_item.h> //点
#include <Item_classification_base.h>
#include <Point_set_item_classification.h>
#include <QFileInfo>
#include <QMessageBox>
#include <QAxObject>
#include <QAxWidget>
#include <QDir>

#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "commonfunctions.h"
#include "setting.hpp"
#include "pctio.h"
#include "mydef.h"
#include "DangerousDistanceCheck.h"

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <vtkOutputWindow.h>
#include <QVTKOpenGLWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <QSurfaceFormat>

//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )  // 隐藏程序
bool ParserCmdline(int argc, char *argv[]);
bool train();
bool classif();
void correct();
void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pcl::PointIndices>& jlClusters, std::vector < pct::LineInfo>& lineClusters, std::vector <pct::TowerInfo>& towerClusters, std::vector <pct::VegetInfo> &vegetClusters);
bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
bool ReadPoscorrectOpts(pct::Setting& setting, boost::program_options::variables_map &vm);
void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointIndicesPtr ground_indices, std::vector <pct::VegetInfo>& vegetClusters, std::vector <pct::LineInfo>& lineClusters, std::vector <pct::TowerInfo>& towerClusters);
void SaveTowers(QString filepath, std::vector <pct::TowerInfo> &towerClusters);
void SaveLines(QString filepath, std::vector <pct::LineInfo> &towerClusters);
void PositionCorrection(QString tower_excel);
void LoadTowers(QString filepath, std::vector <std::tuple<double, double>> &towerClusters);

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(8); //1     

    srand((unsigned int)time(NULL));
    const pct::Setting & setting = pct::Setting::ins();

    std::cout << "ParserCmdline" << std::endl;
    if (false == ParserCmdline(argc, argv))
    {
        return EXIT_FAILURE;
    }
    std::cout << "ParserCmdline" << std::endl;

    if (setting.cmdtype == "train")
    {
        train();
    }
    else if (setting.cmdtype == "classif")
    {
        //classif();
        //correct();
        classif();
        std::string inputfile = setting.outputdir + "\\out.las";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pct::io::Load_las(src_cloud, inputfile);
        std::cout << "离群点过滤" << std::endl;
        //pct::OutlierRemoval(src_cloud);

        // 因为cgal分类结果是整体的，并不能把每个个体提取出来
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pct::LineInfo> lineClusters;
        std::vector <pct::TowerInfo> towerClusters;
        std::vector <pct::VegetInfo> vegetClusters;
        correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);

        SaveTowers(QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "铁塔.xlsx").c_str()) , towerClusters);
        SaveLines(QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "电力线.xlsx").c_str()), lineClusters);
    }
    else if (setting.cmdtype == "distancecheck")
    {
        classif();
        std::string inputfile = setting.outputdir + "\\out.las";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pct::io::Load_las(src_cloud, inputfile);
        std::cout << "离群点过滤" << std::endl;
        //pct::OutlierRemoval(src_cloud);

        // 因为cgal分类结果是整体的，并不能把每个个体提取出来
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        std::vector <pcl::PointIndices> otheCluster;
        std::vector <pct::LineInfo> lineClusters;
        std::vector <pct::TowerInfo> towerClusters;
        std::vector <pct::VegetInfo> vegetClusters;
        correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);
        SaveTowers(QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "铁塔.xlsx").c_str()), towerClusters);
        SaveLines(QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "电力线.xlsx").c_str()), lineClusters);
        std::cout << "correct end：end end" << std::endl;
        checkLinesDistanceDangerous(src_cloud, ground_indices, vegetClusters, lineClusters, towerClusters);

        // 导出pdf
        std::string pdfexe_path = setting.appdir + "PdfReport\\PdfReport.exe";
        std::string json_path = setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "检测结果.json";
        if (!boost::filesystem::exists(boost::filesystem::path(pdfexe_path)))
        {
            std::cout << "未能找到 " << pdfexe_path << "，导出pdf失败！";
            return EXIT_FAILURE;
        }
        else
        {
            std::string cmd_str = pdfexe_path + " --jsonpath " + json_path;
            //system(cmd_str.c_str());
            WinExec(cmd_str.c_str(), SW_HIDE);
            std::cout <<  cmd_str << "\n导出pdf成功！";
        }

    }
    else if (setting.cmdtype == "poscorrect")
    {
        if (setting.reclassif)
        {
            classif();
            std::string inputfile = setting.outputdir + "\\out.las";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pct::io::Load_las(src_cloud, inputfile);
            std::cout << "离群点过滤" << std::endl;
            //pct::OutlierRemoval(src_cloud);

            // 因为cgal分类结果是整体的，并不能把每个个体提取出来
            pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
            std::vector <pcl::PointIndices> otheCluster;
            std::vector <pct::LineInfo> lineClusters;
            std::vector <pct::TowerInfo> towerClusters;
            std::vector <pct::VegetInfo> vegetClusters;
            correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);

            QString tower_excle = QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "铁塔.xlsx").c_str());
            QString line_excel  = QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "电力线.xlsx").c_str());
            std::cout << tower_excle.toLocal8Bit().data() << std::endl;
            std::cout << line_excel.toLocal8Bit().data() << std::endl;

            SaveTowers(tower_excle, towerClusters);
            SaveLines(line_excel, lineClusters);
            std::cout << "全部识别完成" << std::endl;

            PositionCorrection(tower_excle);
        }
        else
        {
            QString tower_excle = QString::fromLocal8Bit((setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + ".xlsx").c_str());
            PositionCorrection(tower_excle);
        }
    }
    else
    {
        return EXIT_FAILURE;
    }
    

    return EXIT_SUCCESS;
}

void PositionCorrection(QString tower_excel)
{
    pct::Setting &setting = pct::Setting::ins();
    bool overrideExcel = setting.overrideExcel;         // 覆盖excel
    bool stampcorrectcell = setting.stampcorrectcell;   // 提示修改
    std::vector <std::tuple<double, double>> towerClusters;
    LoadTowers(tower_excel, towerClusters);
    QString src_tower_dir = QString::fromLocal8Bit(setting.exceldir.c_str());

    QDir dir(src_tower_dir);
    if (!dir.exists())
    {
        std::cout << "excel目录不存在!" << std::endl;
        return;
    }

    QStringList filters;
    filters << QString("*.xlsx");
    dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
    dir.setNameFilters(filters);
    QFileInfoList list = dir.entryInfoList();
    int file_count = list.count();
    if (file_count <= 0)
    {
        std::cout << "excel文件数目小于0!" << std::endl;
        return;
    }
    std::cout << "list" << std::endl;
    QStringList string_list;
    for (int i = 0; i < list.count(); i++)
    {
        QFileInfo file_info = list.at(i);
        QString suffix = file_info.suffix();
        if (QString::compare(suffix, QString("xlsx"), Qt::CaseInsensitive) == 0 && !file_info.fileName().count(QStringLiteral("纠偏")))
        {
            QString absolute_file_path = file_info.absoluteFilePath();
            string_list.append(absolute_file_path);
        }
    }

    std::cout << "string_list" << string_list.size() << std::endl;
    // 遍历原始excels
    for (int i = 0; i < string_list.size(); ++i)
    {
        HRESULT r = OleInitialize(0);
        if (r != S_OK && r != S_FALSE) {
            qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
        }
        QAxObject excel("Excel.Application");
        excel.setProperty("Visible", false); //隐藏打开的excel文件界面
        excel.setProperty("DisplayAlerts", false);//不显示任何警告信息
        QAxObject *workbooks = excel.querySubObject("WorkBooks");
        QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", string_list[i]); //打开文件
        QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //访问第一个工作表
        QAxObject * usedrange = worksheet->querySubObject("UsedRange");
        QAxObject * rows = usedrange->querySubObject("Rows");
        int intRows = rows->property("Count").toInt(); //行数

        QString Range = "A2:I" + QString::number(intRows);
        QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //读取范围
        QVariant allEnvDataQVariant = allEnvData->property("Value");
        QVariantList allEnvDataList = allEnvDataQVariant.toList();
        
        // 遍历单个原始excel每行
        std::cout << "intRows" << intRows << string_list[i].toLocal8Bit().data() << std::endl;
        for (int j = 0; j < intRows-1; j++)
        {
            QVariantList allEnvDataList_j = allEnvDataList[j].toList();
            double j_log = allEnvDataList_j[7].toDouble();
            double j_lat = allEnvDataList_j[8].toDouble();

            // 遍历识别铁塔
            for (int k = 0; k < towerClusters.size(); ++k)
            {
                double offset = pct::getLonDistance(j_log, j_lat, std::get<0>(towerClusters[k]), std::get<1>(towerClusters[k]));
                std::cout << std::fixed << setprecision(7) 
                    << j_log << " " << j_lat << "    "
                    << std::get<0>(towerClusters[k]) << " " << std::get<1>(towerClusters[k]) << " " << offset << std::endl;

                if (offset < 30 && offset > 0.2)    //  30米内误差大于20厘米
                {
                    if (stampcorrectcell)
                    {
                        std::cout << "stampcorrectcell" << std::endl;
                        worksheet->querySubObject("Cells(int,int)", j + 2, 8)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
                        worksheet->querySubObject("Cells(int,int)", j + 2, 9)->querySubObject("Interior")->setProperty("Color", QColor(0, 255, 0));
                    }
                    worksheet->querySubObject("Cells(int,int)", j + 2, 8)->dynamicCall("SetValue(const QVariant&)", std::get<0>(towerClusters[k]));
                    worksheet->querySubObject("Cells(int,int)", j + 2, 9)->dynamicCall("SetValue(const QVariant&)", std::get<1>(towerClusters[k]));
                    //allEnvDataList_j[7].setValue(std::get<0>(towerClusters[k]));
                    //allEnvDataList_j[8].setValue(std::get<1>(towerClusters[k]));
                }
            }
        }
        // 回写入原数据
        //allEnvData->dynamicCall("SetValue(const QVariant&)", QVariant(allEnvDataList));//存储所有数据到 excel 中,批量操作,速度极快
        QString write_path = QDir::toNativeSeparators(QFileInfo(string_list[i]).absoluteFilePath());
        
        if (!overrideExcel)
            write_path = QFileInfo(string_list[i]).absolutePath() + "\\" + QFileInfo(string_list[i]).baseName() + QStringLiteral("纠偏.xlsx");
        else
            write_path = QFileInfo(string_list[i]).absoluteFilePath();
        if (QFile::exists(write_path))
            QFile::remove(write_path);

        workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(write_path));//保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。

        workbook->dynamicCall("Close (Boolean)", false);
        excel.dynamicCall("Quit()");
        OleUninitialize();
    }
}

void LoadTowers(QString filepath, std::vector <std::tuple<double, double>> &towerClusters)
{
    if (!QFile(filepath).exists())
    {
        std::cout << "void LoadTowers()  !QFile(filepath).exists()" << std::endl;
        return;
    }
    HRESULT r = OleInitialize(0);
    if (r != S_OK && r != S_FALSE) {
        qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
    }
    std::cout << "LoadTowers filepath" << filepath.toLocal8Bit().data() << std::endl;
    QAxObject excel("Excel.Application");
    excel.setProperty("DisplayAlerts", false);//不显示任何警告信息
    excel.setProperty("Visible", false); //隐藏打开的excel文件界面
    QAxObject *workbooks = excel.querySubObject("WorkBooks");
    QAxObject *workbook = workbooks->querySubObject("Open(QString, QVariant)", filepath); //打开文件
    QAxObject * worksheet = workbook->querySubObject("WorkSheets(int)", 1); //访问第一个工作表
    QAxObject * usedrange = worksheet->querySubObject("UsedRange");
    QAxObject * rows = usedrange->querySubObject("Rows");
    int intRows = rows->property("Count").toInt(); //行数

    QString Range = "A2:D" + QString::number(intRows);
    QAxObject *allEnvData = worksheet->querySubObject("Range(QString)", Range); //读取范围
    QVariant allEnvDataQVariant = allEnvData->property("Value");
    QVariantList allEnvDataList = allEnvDataQVariant.toList();

    std::cout << "LoadTowers intRows" << intRows << std::endl;
    for (int i = 0; i < intRows - 1; i++)
    {
        QVariantList allEnvDataList_i = allEnvDataList[i].toList();

        double log = allEnvDataList_i[1].toDouble();
        double lat = allEnvDataList_i[2].toDouble();
        //double x = log;
        //double y = lat;
        //pct::LatLon2UTMXY(x, y);
        towerClusters.push_back(std::tuple<double, double>(log, lat));
    }
   
    workbook->dynamicCall("Close (Boolean)", false);
    excel.dynamicCall("Quit()");
    OleUninitialize();
}

void SaveTowers(QString filepath, std::vector <pct::TowerInfo> &towerClusters)
{
    if (!filepath.isEmpty()){
        if (QFile::exists(filepath))
        {
            std::cout << "QFile::exists(filepath)   QFile::remove(filepath);" << std::endl;
            QFile::remove(filepath);
        }
         
        HRESULT r = OleInitialize(0);
        if (r != S_OK && r != S_FALSE) {
            std::cout << "Qt: Could not initialize OLE;" << std::endl;
        }
        QAxObject *excel = new QAxObject("Excel.Application");//连接Excel控件
        excel->dynamicCall("SetVisible (bool Visible)", false);//不显示窗体
        excel->setProperty("DisplayAlerts", false);//不显示任何警告信息。如果为true那么在关闭是会出现类似“文件已修改，是否保存”的提示
        QAxObject *workbooks = excel->querySubObject("WorkBooks");//获取工作簿集合
        workbooks->dynamicCall("Add");//新建一个工作簿
        QAxObject *workbook = excel->querySubObject("ActiveWorkBook");//获取当前工作簿
        QAxObject *worksheets = workbook->querySubObject("Sheets");//获取工作表集合
        QAxObject *worksheet = worksheets->querySubObject("Item(int)", 1);//获取工作表集合的工作表1，即sheet1

        QList<QVariant> allRowsData;//保存所有行数据
        QList<QVariant> headRowData;//excel头

        headRowData.append(QVariant(QStringLiteral("编号")));
        headRowData.append(QVariant(QStringLiteral("经度")));
        headRowData.append(QVariant(QStringLiteral("纬度")));
        headRowData.append(QVariant(QStringLiteral("高程值")));
        allRowsData.append(QVariant(headRowData));
        pct::TowerInfo *tower_iter;
        
        for (int row = 1; row <= towerClusters.size(); row++)
        {
            tower_iter = &towerClusters[row - 1];
            double x = tower_iter->cen.x;
            double y = tower_iter->cen.y;
            QList<QVariant> aRowData;//保存一行数据
            pct::UTMXY2LatLon(x, y);

            aRowData.append(QVariant(QString::fromLocal8Bit(tower_iter->getNo().c_str()).remove('#')));
            aRowData.append(QVariant(x));
            aRowData.append(QVariant(y));
            aRowData.append(QVariant(tower_iter->max.z));
            allRowsData.append(QVariant(aRowData));
        }

        QAxObject *range = worksheet->querySubObject("Range(const QString )", QString("A1:D") + QString::number(towerClusters.size()+1));
        range->dynamicCall("SetValue(const QVariant&)", QVariant(allRowsData));//存储所有数据到 excel 中,批量操作,速度极快
        range->setProperty("HorizontalAlignment", -4108);
        range->setProperty("VerticalAlignment", -4108);
        QAxObject * cells = range->querySubObject("Columns");
        cells->dynamicCall("AutoFit");

        range->querySubObject("Font")->setProperty("Size", 14);//设置字号

        std::cout << QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()).toLocal8Bit().data() << std::endl;
        workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()));//保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。
        workbook->dynamicCall("Close (Boolean)", false);
        excel->dynamicCall("Quit()");//关闭excel
        delete excel;
        excel = NULL;
        OleUninitialize();
    }
    std::cout << filepath.toLocal8Bit().data() << std::endl;
}

void SaveLines(QString filepath, std::vector <pct::LineInfo> &lineClusters)
{
    std::cout << filepath.toLocal8Bit().data() << std::endl;
    if (!filepath.isEmpty()){
        if (QFile::exists(filepath))
            QFile::remove(filepath);
        HRESULT r = OleInitialize(0);
        if (r != S_OK && r != S_FALSE) {
            qWarning("Qt: Could not initialize OLE (error %x)", (unsigned int)r);
        }
        QAxObject *excel = new QAxObject("Excel.Application");//连接Excel控件
        excel->dynamicCall("SetVisible (bool Visible)", false);//不显示窗体
        excel->setProperty("DisplayAlerts", true);//不显示任何警告信息。如果为true那么在关闭是会出现类似“文件已修改，是否保存”的提示
        QAxObject *workbooks = excel->querySubObject("WorkBooks");//获取工作簿集合
        workbooks->dynamicCall("Add");//新建一个工作簿
        QAxObject *workbook = excel->querySubObject("ActiveWorkBook");//获取当前工作簿
        QAxObject *worksheets = workbook->querySubObject("Sheets");//获取工作表集合
        QAxObject *worksheet = worksheets->querySubObject("Item(int)", 1);//获取工作表集合的工作表1，即sheet1

        QList<QVariant> allRowsData;//保存所有行数据
        QList<QVariant> headRowData;//excel头

        headRowData.append(QVariant(QStringLiteral("编号")));
        headRowData.append(QVariant(QStringLiteral("起始经度")));
        headRowData.append(QVariant(QStringLiteral("起始纬度")));
        headRowData.append(QVariant(QStringLiteral("起始高程")));
        headRowData.append(QVariant(QStringLiteral("终点经度")));
        headRowData.append(QVariant(QStringLiteral("终点纬度")));
        headRowData.append(QVariant(QStringLiteral("终点高程")));
        headRowData.append(QVariant(QStringLiteral("水平长度")));
        allRowsData.append(QVariant(headRowData));
        pct::LineInfo *line_iter;
        
        for (int row = 1; row <= lineClusters.size(); row++)
        {
            line_iter = &lineClusters[row - 1];
            double stax = line_iter->sta.x;
            double stay = line_iter->sta.y;
            double endx = line_iter->end.x;
            double endy = line_iter->end.y;
            QList<QVariant> aRowData;//保存一行数据
            pct::UTMXY2LatLon(stax, stay);
            pct::UTMXY2LatLon(endx, endy);

            aRowData.append(QVariant(QString::fromLocal8Bit(line_iter->getLineNo().c_str()).remove('#').replace('-','_')));
            aRowData.append(QVariant(stax));
            aRowData.append(QVariant(stay));
            aRowData.append(QVariant(line_iter->sta.z));
            aRowData.append(QVariant(endx));
            aRowData.append(QVariant(endy));
            aRowData.append(QVariant(line_iter->end.z));
            aRowData.append(QVariant(pct::Distance2d(line_iter->sta.x, line_iter->sta.y, line_iter->end.x, line_iter->end.y)));
            allRowsData.append(QVariant(aRowData));
        }

        QAxObject *range = worksheet->querySubObject("Range(const QString )", QString("A1:H") + QString::number(lineClusters.size() + 1));
        range->dynamicCall("SetValue(const QVariant&)", QVariant(allRowsData));//存储所有数据到 excel 中,批量操作,速度极快
        range->setProperty("HorizontalAlignment", -4108);
        range->setProperty("VerticalAlignment", -4108);
        QAxObject * cells = range->querySubObject("Columns");
        cells->dynamicCall("AutoFit");

        range->querySubObject("Font")->setProperty("Size", 14);//设置字号

        std::cout << QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()).toLocal8Bit().data() << std::endl;
        workbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(QFileInfo(filepath).absoluteFilePath()));//保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。
        workbooks->dynamicCall("Close (boolean)", false);
        excel->dynamicCall("Quit()");//关闭excel
        delete excel;
        excel = NULL;
        OleUninitialize();
    }
}

void checkLinesDistanceDangerous(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pct::VegetInfo>& vegetClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters)
{
    std::cout << "DangerousDistanceCheck" << std::endl;
    DangerousDistanceCheck ddc;
    ddc.setData(src_cloud 
        , ground_indices
        , vegetClusters
        , lineClusters
        , towerClusters
       );
    ddc.TooNearCheck();
    ddc.showNearCheck();
    std::cout << "showNearCheck endl." << std::endl;
}

bool ReadyTrainOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    setsettingitem(classdir, std::string, true, "");
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir)) //如果文件夹不存在，或者不是文件夹
    {
        std::cout << "选项classdir文件夹路径错误！" << std::endl;
        std::cout << classdir << std::endl;
        return false;
    }
    setsettingitem(gridsize, float, false, 0.3);

    return true;
}

bool ReadyClassifOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    setsettingitem(inputfile, std::string, true, "");
    boost::filesystem::path path_file(inputfile);
    boost::filesystem::path path_dir(path_file.parent_path().string());

    if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file) || boost::filesystem::extension(path_file) != ".las")
    {
        std::cout << "选项inputfile文件路径错误！" << std::endl;
        std::cout << inputfile << std::endl;
        return false;
    }

    setsettingitem(outputdir, std::string, false, path_dir.string() + "\\" + path_file.stem().string());
    boost::filesystem::path out_dir = boost::filesystem::path(outputdir);
    if (!boost::filesystem::exists(out_dir))
    {
        boost::filesystem::create_directories(out_dir);
    }
    if (!boost::filesystem::is_directory(out_dir))
    {
        std::cout << "选项outputdir目录错误！" << std::endl;
        std::cout << outputdir << std::endl;
        return false;
    }

    setsettingitem(classdir, std::string, true, "");
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //如果文件夹不存在，或者不是文件夹
    {
        std::cout << "样本文件夹中没有训练文件config.xml，请先训练样本！" << std::endl;
        std::cout << classdir << std::endl;
        return false;
    }
    setsettingitem(method, int, false, 2);

    setsettingitem(gridsize, float, false, 0.3);

    return true;
}

bool ReadyDistancecheckOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    if (!ReadyClassifOpts(setting, vm))
    {
        return false;
    }

    return true;
}

bool ReadPoscorrectOpts(pct::Setting& setting, boost::program_options::variables_map &vm)
{
    std::cout << "ReadPoscorrectOpts" << std::endl;
    setsettingitem(inputfile, std::string, true, "");

    boost::filesystem::path path_file(inputfile);
    boost::filesystem::path path_dir(path_file.parent_path().string());

    std::cout << "boost::filesystem::extension(path_file)" << std::endl;
    if (boost::filesystem::extension(path_file) == ".xlsx")
    {
        setting.reclassif = false;
    }
    else if (boost::filesystem::extension(path_file) == ".las")
    {
        setting.reclassif = true;
    }
    else
    {
        std::cout << "选项inputfile文件路径错误！" << std::endl;
        std::cout << inputfile << std::endl;
        return false;
    }
    std::cout << "setting.reclassif" << setting.reclassif << std::endl;

    if (!boost::filesystem::exists(path_file) || !boost::filesystem::is_regular_file(path_file))
    {
        std::cout << "选项inputfile文件路径错误！" << std::endl;
        std::cout << inputfile << std::endl;
        return false;
    }

    std::cout << path_dir.string() + "\\" + path_file.stem().string() << std::endl;
    if (setting.reclassif == true)
    {
        setsettingitem(outputdir, std::string, false, path_dir.string() + "\\" + path_file.stem().string());
        boost::filesystem::path out_dir = boost::filesystem::path(outputdir);
        if (!boost::filesystem::exists(out_dir))
        {
            boost::filesystem::create_directories(out_dir);
        }
        if (!boost::filesystem::is_directory(out_dir))
        {
            std::cout << "选项outputdir目录错误！" << std::endl;
            std::cout << outputdir << std::endl;
            return false;
        }
    }
    else
    {
        setting.outputdir = path_dir.string();
    }

    setsettingitem(exceldir, std::string, true, "");
    boost::filesystem::path excel_dir = boost::filesystem::path(exceldir);
    if (!boost::filesystem::exists(excel_dir) || !boost::filesystem::is_directory(excel_dir))
    {
        std::cout << "选项excel_dir目录错误！" << std::endl;
        std::cout << "excel_dir" << excel_dir << std::endl;
        return false;
    }
    
    setsettingitem(classdir, std::string, true, "");
    if (!boost::filesystem::exists(classdir) || !boost::filesystem::is_directory(classdir) || _access((classdir + "\\config.xml").c_str(), 0) == -1) //如果文件夹不存在，或者不是文件夹
    {
        std::cout << "样本文件夹中没有训练文件config.xml，请先训练样本！" << std::endl;
        std::cout << classdir << std::endl;
        return false;
    }
    setsettingitem(method, int, false, 2);

    setsettingitem(gridsize, float, false, 0.3);

    //setsettingitem(reclassif, bool, false, true);

    setsettingitem(overrideExcel, bool, false, false)

    setsettingitem(stampcorrectcell, bool, false, true);


    return true;
}

bool ParserCmdline(int argc, char *argv[])
{
    std::cout << "ParserCmdline entry" << std::endl;
    std::string exe_name = pct::GetExeName();
    pct::Setting& setting = pct::Setting::ins();
    // 解析命令行
    boost::program_options::variables_map vm;
     boost::program_options::options_description opts("pointcloud tool options"
         "\n示例："
         "\n（1）" + exe_name + " --cmdtype train --classdir classdir --gridsize 0.3"
         "\n（2）" + exe_name + " --cmdtype classif --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n（3）" + exe_name + " --cmdtype distancecheck --inputfile inputfile --outputdir outputdir --classdir classdir --method 2"
         "\n（4）" + exe_name + " --cmdtype poscorrect --inputfile inputfile --overrideExcel false --stampcorrectcell true  --exceldir \"铁塔纠偏数据\"  --outputdir outputdir --classdir classdir --method 2"
         "\n参数");

    opts.add_options()
        ("cmdtype", boost::program_options::value<std::string>(), "命令类型｛train classif distancecheck｝")
        ("inputfile", boost::program_options::value<std::string>(), "点云路径｛*.las｝")
        ("classdir", boost::program_options::value<std::string>(), "样本文件目录 <dir>")
        ("outputdir", boost::program_options::value<std::string>(), "输出结果目录 [dir]")
        ("method", boost::program_options::value<int>(), "分类方法 [int] 0-普通 1-平滑 2-非常平滑")
        ("gridsize", boost::program_options::value<float>(), "叶节点尺寸 [float]")
        //("reclassif", boost::program_options::value<bool>(), "是否重新分类 [bool]")
        ("exceldir", boost::program_options::value<std::string>(), "纠偏塔数据目录 [std::string]")
        ("overrideExcel", boost::program_options::value<bool>(), "是否覆盖源excel [bool]")
        ("stampcorrectcell", boost::program_options::value<bool>(), "是否标记修改cell [bool]")
        ("help", "帮助");

    try{
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    }
    catch (...){
        std::cout << "输入的参数中存在未定义的选项！\n";
        return false;
    }

    // 检查并赋值
    if (vm.count("help")){//若参数中有help选项
        std::cout << opts << std::endl;
        return false;
    }
    else
    {
        setsettingitem(cmdtype, std::string, true, "");
        if (cmdtype == "train")
        {
            if (!ReadyTrainOpts(setting, vm))
                return false;
        }
        else if (cmdtype == "classif")
        {
            if (!ReadyClassifOpts(setting, vm))
                return false;
        }
        else if (cmdtype == "distancecheck")
        {
            if (!ReadyDistancecheckOpts(setting, vm))
                return false;
        }
        else if (cmdtype == "poscorrect")
        {
            std::cout << "ReadPoscorrectOpts start" << std::endl;
            if (!ReadPoscorrectOpts(setting, vm))
                return false;
        }
        else
        {
            std::cout << "参数cmdtype输入错误" << std::endl;
            return false;
        }
    }

    return true;
}

bool train()
{
    std::cout << "重新训练..." << std::endl;
    const pct::Setting & setting = pct::Setting::ins();
    int nb_scales = setting.value<int>("nb_scales");
    int nb_trials = setting.value<int>("nb_trials");
    int simplify_model = setting.value<int>("simplify");
    // 遍历样本文件夹
    boost::filesystem::path yangbendir(setting.classdir);
    std::cout << "setting.classdir：" << setting.classdir << std::endl;
    boost::filesystem::directory_iterator iter_dirend;

    std::vector<std::string> labels;
    std::vector<std::string> labelxmls;
    for (boost::filesystem::directory_iterator yangbendiriter(yangbendir); yangbendiriter != iter_dirend; ++yangbendiriter)
    {
        // 遍历类型下边的所有样本文件
        if (boost::filesystem::is_directory(*yangbendiriter))
        {
            std::vector<std::string> lasxmls;
            std::string classdir = yangbendiriter->path().string(); // 得到类文件夹路径
            std::string classname = yangbendiriter->path().stem().string();  // 得到类名
            std::string labelxml = classdir + "\\config.xml";
            std::cout << "开始训练" << classname << "..." << std::endl;
            for (boost::filesystem::directory_iterator fileiter(*yangbendiriter); fileiter != iter_dirend; ++fileiter)
            {
                // 如果文件有效，而且后缀名为las，则是我们要训练的东西
                if (boost::filesystem::is_regular_file(*fileiter) && boost::algorithm::to_lower_copy(boost::filesystem::extension(*fileiter))== ".las")
                {
                    std::string laspath = fileiter->path().string(); // 得到文件路径
                    std::string lasname = fileiter->path().stem().string(); // 得到文件名
                    
                    // 抽稀  
                    std::ostringstream tempfile;
                    tempfile << setting.outputdir << "\\" << lasname << "_simple.las";
                    if (boost::filesystem::exists(boost::filesystem::path(tempfile.str())))
                        boost::filesystem::remove(boost::filesystem::path(tempfile.str()));
                   

                    pct::simple(laspath, tempfile.str(), setting.gridsize, simplify_model);
                    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));
                    

                    std::cout << "文件：" << lasname << "..." << std::endl;
                    if (scene_item && scene_item->point_set()->check_colors())
                    {
                        auto start = std::chrono::system_clock::now();
                        Point_set* points = scene_item->point_set();
                        // 计算特征
                        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
                        classif->compute_features(nb_scales);

                        // 添加label
                        classif->add_new_label(classname.c_str());
                        classif->add_new_label(unselect_str);

                       // 选择红色的
                        classif->change_color(0);
                        std::vector<Point_set::Index> unselected, selected;
                        for (Point_set::Index idx : *points)
                        {
                            if (points->red(idx) == 1. && scene_item->point_set()->green(idx) == 0. && scene_item->point_set()->blue(idx) == 0.)
                                selected.push_back(idx);
                            else
                                unselected.push_back(idx);
                        }

                        // 选择,加入label
                        scene_item->selectPoints(selected, unselected);
                        std::cout << "selected red points： " << points->number_of_removed_points() << std::endl;
                        classif->add_selection_to_training_set(0);  

                        // 非选择,加入unselectlabel
                        scene_item->selectPoints(unselected, selected);
                        //scene_item->invertSelection();  // 切记add_selection_to_training_set会删除选择集，所以这个函数在这里不能用
                        std::cout << "unselect points： " << points->number_of_removed_points() << std::endl;
                        classif->add_selection_to_training_set(1);

                        // 训练
                        std::cout << "训练" << classname << "-" << lasname << std::endl;
                        classif->train(0, nb_trials, 0, 0);
                        std::string lasxml_path = classdir + "\\" + lasname + ".xml";
                        classif->save_config(lasxml_path.c_str(), 0);
                        lasxmls.push_back(lasxml_path);

                        std::cout << "训练" << classname << "完成。一共" << points->size() << "个点,选择" << selected.size() << "个。\n用时"
                            << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() << "s" << std::endl;
                     }
                }
            }
            
            if (lasxmls.size())
            {
                std::cout << lasxmls.size() << std::endl;
                pct::combineTrainXmlFiles(lasxmls, labelxml);
                labelxmls.push_back(labelxml);
            } 
        }
    }

    std::cout << labelxmls.size() << std::endl;
    if (labelxmls.size())
        pct::combineTrainXmlFiles(labelxmls, setting.classdir + "\\config.xml");
    return true;
}

bool classif()
{
    std::cout << "开始分类..." << std::endl;
    const pct::Setting & setting = pct::Setting::ins();
    int nb_scales = setting.value<int>("nb_scales");
    int simplify_model = setting.value<int>("simplify");
    int method = setting.method;
    std::cout << "method" << method << std::endl;
    std::cout << "simplify_model" << simplify_model << std::endl;
    std::string labelname_traverse;
    std::string config_xml = setting.classdir + "\\config.xml";


    // 抽稀  
    std::ostringstream tempfile;
    tempfile << setting.outputdir << "\\" << "simple.las";
    if (boost::filesystem::exists(boost::filesystem::path(tempfile.str())))
        boost::filesystem::remove(boost::filesystem::path(tempfile.str()));
    pct::simple(setting.inputfile, tempfile.str(), setting.gridsize, simplify_model);
    boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(tempfile.str()));

    if (scene_item)
    {
        // 计算特征
        boost::shared_ptr<Point_set_item_classification> classif(new Point_set_item_classification(scene_item.get()));
        classif->compute_features(nb_scales);

        // 添加label
        std::string labelname;
        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(config_xml, pt);
        BOOST_AUTO(labels, pt.get_child("classification.labels"));
        for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
        {
            labelname_traverse = label->second.get<std::string>("name");
            classif->add_new_label(labelname_traverse.c_str(), setting.cls_intcolor(labelname_traverse));
        }

        // 分类
        classif->load_config(config_xml.c_str(), 0);
        classif->run(method, 0, 16, 0.5);
        pct::io::lassave(scene_item.get(), setting.outputdir + "\\out.las");
    }
    return true;
}

void correct()
{
    const pct::Setting & setting = pct::Setting::ins();
    std::string inputfile = setting.outputdir + "\\out.las";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pct::io::Load_las(src_cloud, inputfile);
    std::cout << "离群点过滤" << std::endl;
    //pct::OutlierRemoval(src_cloud);

    // 因为cgal分类结果是整体的，并不能把每个个体提取出来
    std::vector <pcl::PointIndices> otheCluster;
    std::vector < pct::LineInfo> lineClusters;
    std::vector <pct::TowerInfo> towerClusters;
    std::vector <pct::VegetInfo> vegetClusters;
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    correct(src_cloud, ground_indices, otheCluster, lineClusters, towerClusters, vegetClusters);
}

void correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
    pcl::PointIndicesPtr ground_indices,
    std::vector <pcl::PointIndices>& jlClusters,
    std::vector <pct::LineInfo>& lineClusters,
    std::vector <pct::TowerInfo>& towerClusters,
    std::vector <pct::VegetInfo> &vegetClusters)
{
    const pct::Setting & setting = pct::Setting::ins();
   
    std::string outputfile = setting.outputdir + "\\out.las";
    const int tower_intersectline_threshold = 3;

    std::cout << "correct begin" << std::endl;
    pct::io::save_las(src_cloud, setting.outputdir + "\\cgalclassif.las");


    // 提取地面点索引
    pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
    pct::ExtractGround(src_cloud, cloud_indices, ground_indices);
    std::cout << "非地面点：" << cloud_indices->indices.size()
        << "地面点：" << ground_indices->indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(ground_indices);
    extract.filter(*ground);


    // 再对剩余的点颜色聚类
    pct::colorClusters(src_cloud, *cloud_indices, jlClusters);
    std::cout << "聚类数量：" << jlClusters.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*src_cloud, *tmpcloud);
    pcl::PointXYZRGB *tmppt1;

    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        unsigned char r = rand() % 256;
        unsigned char g = rand() % 256;
        unsigned char b = rand() % 256;

        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt1 = &tmpcloud->at(*itt);
            tmppt1->r = r;
            tmppt1->g = g;
            tmppt1->b = b;
        }
    }
    for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
    {
        tmppt1 = &tmpcloud->at(*it);
        tmppt1->r = 0;
        tmppt1->g = 0;
        tmppt1->b = 255;
    }
    pct::io::save_las(tmpcloud, setting.outputdir + "\\pcljl.las");
    tmpcloud.reset();

	std::cout << "电力线提取begin"  << std::endl;
    // 电力线提取
    // 聚类结果是否含有70%以上的电力线点，如果是，就说明是电力线点
    for (auto it = jlClusters.begin(); it != jlClusters.end(); )
    {
		std::cout << "电力线提取1" << std::endl;
        if (pct::pointsCountsForColor(src_cloud, *it, setting.cls_intcolor(power_line_str)) > it->indices.size()*0.25)  // power_line":"255, 255, 0
        {
			std::cout << "电力线提取2" << std::endl;
            pct::LineInfo line = pct::lineInfoFactory(src_cloud, *it);
			std::cout << "电力线提取begin" << std::endl;
            if (pct::LikePowerLine1(ground, line, 10, 0.1, 0.5, 0.5))
            {
				std::cout << "电力线提取3" << std::endl;
                lineClusters.push_back(line);
                it = jlClusters.erase(it);
                continue;
            }
			std::cout << "电力线提取4" << std::endl;
        }
        ++it;
    }
    std::cout << "other数量：" << jlClusters.size() << "电力线识别数量：" << lineClusters.size() << std::endl;

    // 铁塔提取，如果某个聚类与3条电力线相交小于2米，则认为是铁塔
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    for (auto it = jlClusters.begin(); it != jlClusters.end(); )
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr unknowclass(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
        extract.filter(*unknowclass);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> unknowclass_kdtree;
        unknowclass_kdtree.setInputCloud(unknowclass);
        int insrt_size = 0;

        // 遍历每条电力线，是否与当前聚类相距小于一米
        for (int j = 0; j < lineClusters.size(); ++j)
        {
            // 遍历聚类的每一个点
            for (int k = 0; k < lineClusters[j].indices.indices.size(); ++k)
            {
                if (unknowclass_kdtree.radiusSearch(src_cloud->at(lineClusters[j].indices.indices[k]), 2, indices, sqr_distances))
                {
                    insrt_size++;
                    break;
                }
            }
        }
        if (insrt_size >= tower_intersectline_threshold)  // 有3根电力线和他相交了，基本可以确定他就是铁塔了
        {
            towerClusters.push_back(pct::TowerInfo(*it));
            it = jlClusters.erase(it);
        }
        else
        {
            ++it;
        }
    }
    // 合并铁塔
    std::cout << "合并铁塔前other数量：" << jlClusters.size() << "电力线识别数量：" << lineClusters.size() << "铁塔识别数量：" << towerClusters.size() << std::endl;
    pct::MergeTower(src_cloud, ground_indices, jlClusters, lineClusters, towerClusters);
    std::cout << "合并铁塔前other数量：" << jlClusters.size() << "电力线识别数量：" << lineClusters.size() << "铁塔识别数量：" << towerClusters.size() << std::endl;


    // 计算电力线和铁塔编号
    for (int i = 0; i < towerClusters.size(); i++)
    {
        pct::getMinMax3D(*src_cloud, towerClusters[i].indices, towerClusters[i].min, towerClusters[i].max);
        pcl::PointXYZRGB midpt;
        
        midpt.x = (towerClusters[i].min.x + towerClusters[i].max.x) / 2;
        midpt.y = (towerClusters[i].min.y + towerClusters[i].max.y) / 2;
        midpt.z = (towerClusters[i].min.z + towerClusters[i].max.z) / 2;
        towerClusters[i].cen = midpt;
    }
    std::cout << "计算铁塔中心点完成" << std::endl;
    pcl::PointXYZRGB temppt;
    pct::TowerInfo tempcluster;
    for (int i = 0; i < (int)towerClusters.size()-1; i++)
    {
        std::cout << "铁塔排序" << i << std::endl;
        pcl::PointXYZRGB minpt, maxpt;
        pct::getMinMax3D(*src_cloud, towerClusters[i].indices, minpt, maxpt);
        std::cout << "获取包围盒" << i << std::endl;
        for (int j = i + 1; j < (int)towerClusters.size() - 1; j++)
        {
            std::cout << "获取包围盒j" << i << " " << j << std::endl;
            pcl::PointXYZRGB minptj, maxptj;
            pct::getMinMax3D(*src_cloud, towerClusters[j].indices, minptj, maxptj);
            std::cout << "获取包围盒j" << minptj << std::endl;
            if (minpt.x < minptj.x)
            {
                std::cout << "移动铁塔j" << i << " " << j << std::endl;
                tempcluster = towerClusters[i];
                towerClusters[i] = towerClusters[j];
                towerClusters[j] = tempcluster;
            }
            std::cout << "移动铁塔j完成" << i << " " << j << std::endl;
        }
        towerClusters[i].tower_no = i+1;
        std::cout << "铁塔排序完成" << i << std::endl;
    }
    std::cout << "铁塔编号完成" << std::endl;
    if (towerClusters.size())
        towerClusters[(int)towerClusters.size() - 1].tower_no = towerClusters.size();

    for (int i = 0; i < lineClusters.size(); ++i)
    {
        pct::LineInfo &line = lineClusters[i];
        for (int j = 0; j < towerClusters.size(); ++j)
        {
            if (pct::Distance2d(line.sta, towerClusters[j].cen) < 25)
            {
                line.begin_tower_no = towerClusters[j].tower_no;
            }
            else if (pct::Distance2d(line.end, towerClusters[j].cen) < 25)
            {
                line.end_tower_no = towerClusters[j].tower_no;
            }
        }
    }
    std::cout << "电力线编号完成" << std::endl;

    //  提取植被
    pcl::KdTreeFLANN<pcl::PointXYZRGB> ground_kdtree;
    if (ground->size())
    {
        ground_kdtree.setInputCloud(ground);
        indices.clear();
        sqr_distances.clear();
        for (auto it = jlClusters.begin(); it != jlClusters.end();)
        {
            pct::VegetInfo veg(src_cloud, *it);
            // 最低点与离地高度<10并且大于30个点，有可能是植物！
            if (it->indices.size() > 30 && ground_kdtree.radiusSearch(veg.min.z, 10, indices, sqr_distances) > 0)
            {
                vegetClusters.push_back(veg);
                it = jlClusters.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    std::cout << "other数量：" << jlClusters.size() << "电力线识别数量：" << lineClusters.size() << "铁塔识别数量：" << towerClusters.size() 
        << "植被数量：" << vegetClusters.size() << "\n一共" << jlClusters.size() + lineClusters.size() + towerClusters.size() + vegetClusters.size() << std::endl;


    //// 全部点都默认黑色
    //for (auto it = src_cloud->begin(); it != src_cloud->end(); ++it)
    //{
    //    it->r = 0;
    //    it->g = 0;
    //    it->b = 0;
    //}

    // 地面点上色
    unsigned int color = setting.cls_intcolor(ground_str);
    unsigned char r = *(((unsigned char *)&color) + 2);
    unsigned char g = *(((unsigned char *)&color) + 1);
    unsigned char b = *(((unsigned char *)&color) + 0);
    pcl::PointXYZRGB *tmppt;
    for (auto it = ground_indices->indices.begin(); it != ground_indices->indices.end(); ++it)
    {
        tmppt = &src_cloud->at(*it);
        tmppt->r = r;
        tmppt->g = g;
        tmppt->b = b;
    }

    // 电力线上色
    color = setting.cls_intcolor(power_line_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
   
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }
    // 铁塔上色
    color = setting.cls_intcolor(tower_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
        std::cout << "检测到铁塔聚类，点数为：" << it->indices.indices.size() << std::endl;
    }

    // 植被点上色
    color = setting.cls_intcolor(veget_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
    {
        for (auto itt = it->indices.indices.begin(); itt != it->indices.indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }

    // 其他点上色
    color = setting.cls_intcolor(others_str);
    r = *(((unsigned char *)&color) + 2);
    g = *(((unsigned char *)&color) + 1);
    b = *(((unsigned char *)&color) + 0);
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        for (auto itt = it->indices.begin(); itt != it->indices.end(); ++itt)
        {
            tmppt = &src_cloud->at(*itt);     
            tmppt->r = r;
            tmppt->g = g;
            tmppt->b = b;
        }
    }

    pct::io::save_las(src_cloud, outputfile);
   



    // 保存电力线 .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lines_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = lineClusters.begin(); it != lineClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        pcl::PointXYZRGB *temp_pt;
        unsigned char r = rand() % 256;
        unsigned char g = rand() % 256;
        unsigned char b = rand() % 256;

        for (int j = 0; j < temp_cloud->size(); ++j)
        {
            temp_pt = &temp_cloud->at(j);
            temp_pt->r = r;
            temp_pt->g = g;
            temp_pt->b = b;
        }

        *lines_cloud += *temp_cloud;
    }
    pct::io::save_las(lines_cloud, setting.outputdir + "\\lines.las");
    lines_cloud.reset();

    // 保存铁塔 .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tower_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = towerClusters.begin(); it != towerClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        *tower_cloud += *temp_cloud;
    }
    pct::io::save_las(tower_cloud, setting.outputdir + "\\towers.las");
    tower_cloud.reset();



    // 保存vegets .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vegets_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = vegetClusters.begin(); it != vegetClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices.indices));
        extract.filter(*temp_cloud);
        *vegets_cloud += *temp_cloud;
    }
    pct::io::save_las(vegets_cloud, setting.outputdir + "\\vegets.las");
    vegets_cloud.reset();

    // 保存others .las
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr others_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto it = jlClusters.begin(); it != jlClusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(src_cloud);
        extract.setIndices(boost::make_shared<std::vector<int>>(it->indices));
        extract.filter(*temp_cloud);
        *others_cloud += *temp_cloud;
    }
    pct::io::save_las(others_cloud, setting.outputdir + "\\others.las");
    others_cloud.reset();

    // 保存ground .las
    ground->clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    extract.setInputCloud(src_cloud);
    extract.setIndices(boost::make_shared<std::vector<int>>(ground_indices->indices));
    extract.filter(*ground_cloud);
    pct::io::save_las(ground_cloud, setting.outputdir + "\\ground.las");
    ground_cloud.reset();
    std::cout << "correct end：end" << std::endl;
}

