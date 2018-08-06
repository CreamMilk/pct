#pragma once

#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

class QRendView : public QVTKWidget
{
    Q_OBJECT

public:
    QRendView(QWidget *parent = Q_NULLPTR);
    ~QRendView();
    void BindCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
        , pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected);

    void UpdateView(bool reset_cloud = false) ;
    pcl::visualization::PCLVisualizer::Ptr Viewer() const;

public slots:
void AllowOperSlot(bool allow);
protected:
    void keyPressEvent(QKeyEvent *event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_;
    std::atomic<bool> allow_;
};
