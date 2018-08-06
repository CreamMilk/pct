#pragma once

#include <QtWidgets/QWidget>
#include <QKeyEvent>
#include <QMutex>
#include "ui_qcolortool.h"
#include <atomic> 
#include <pcl/visualization/pcl_visualizer.h>

class QColorTool : public QWidget
{
    Q_OBJECT

public:
    enum SelectFlag{ NEW = 0, UNION, INTER, DIFF };
    QColorTool(QWidget *parent = Q_NULLPTR);
    void CleanSelects();
    void AllowOper(bool allow);

    static void AreaPickingCallback(const pcl::visualization::AreaPickingEvent& event, void* args);
    static QMutex operaing_;
protected:
    void ConnectUi();
    virtual void keyPressEvent(QKeyEvent *event);
signals:
    void AreaPickingSignal(boost::shared_ptr<std::vector<int>> indices);
    

public slots:
void on_Load_las();
void on_save_las();
void on_select_new();
void on_select_union();
void on_select_inter();
void on_select_diff();
void on_simple();
void on_color();
void on_delete();
void AreaPickingSlot(std::set<int> &indices);

private:
    Ui::QColorToolClass ui;
    QString app_path_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selects_;
    std::set<int, std::greater<int>> selects_index_;
    std::atomic_uint32_t select_flag_;
};
