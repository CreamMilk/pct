#include "qrendview.h"
#include <QApplication>
#include "vtkRenderWindow.h"
#include <pcl/common/common.h>

QRendView::QRendView(QWidget *parent)
    : QVTKWidget(parent)
    ,allow_(true)
{
    viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("viewer", false);
    viewer_->setBackgroundColor(1, 1, 1);
     SetRenderWindow(viewer_->getRenderWindow());
     QVTKInteractor* Interactor = GetInteractor();
     viewer_->setupInteractor(Interactor, viewer_->getRenderWindow());
}

QRendView::~QRendView()
{
    // 这里很坑，你得替它删，不然这里就内存泄漏了
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
}

void QRendView::UpdateView(bool reset_cloud/* = false*/) 
{
//      if (reset_cloud)
//      {
         viewer_->removeAllPointClouds();
         viewer_->addPointCloud(cloud_, "cloud");
         viewer_->addPointCloud(selected_, "cloud_selected");
//      }
//      else
//      {
//          viewer_->updatePointCloud(cloud_, "cloud");
//          viewer_->updatePointCloud(selected_, "cloud_selected");
//      }

    std::stringstream points_ss;
    points_ss << "points_count: " << cloud_->size();
    if (!viewer_->updateText(points_ss.str(), 5, 20, 0, 0.5, 0, std::string("points_count")))
    {
        viewer_->addText(points_ss.str(), 5, 20, 0, 0.5, 0, std::string("points_count"));
    }

    std::stringstream points_select_ss;
    points_select_ss << "select_count: " << selected_->size();
    if (!viewer_->updateText(points_select_ss.str(), 5, 10, 0, 0.5, 0, std::string("select_count")))
    {
        viewer_->addText(points_select_ss.str(), 5, 10, 0, 0.5, 0, std::string("select_count"));
    }
    update();
}

pcl::visualization::PCLVisualizer::Ptr QRendView::Viewer() const
{
    return viewer_;
}

void QRendView::BindCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected)
{
    cloud_ = cloud;
    selected_ = selected;
    viewer_->addPointCloud(cloud_, "cloud");
    viewer_->addPointCloud(selected_, "cloud_selected");
}

void QRendView::keyPressEvent(QKeyEvent  *event)
{
    if (allow_)
    {
        event->ignore();
        QVTKWidget::keyPressEvent(event);
    }
}

void QRendView::AllowOperSlot(bool allow)
{
    allow_ = allow;
    if (!allow)
    {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    }
    else
    {
        QApplication::restoreOverrideCursor();
    }
}

void QRendView::mouseReleaseEvent(QMouseEvent* event)
{
     if (allow_)
     {
         QVTKWidget::mouseReleaseEvent(event);
     }
}

void QRendView::mousePressEvent(QMouseEvent* event)
{
    if (allow_)
    {
        QVTKWidget::mousePressEvent(event);
     }
}
