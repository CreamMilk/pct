#include "qcolortool.h"
#include <lasreader.hpp>
#include <laswriter.hpp>
#include <QFileDialog>
#include <QColorDialog>
#include <QMessageBox>
#include <pcl/filters/uniform_sampling.h>   //均匀采样
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <vtkEventQtSlotConnect.h>
#include <pcl/filters/voxel_grid.h>

QMutex QColorTool::operaing_;

QColorTool::QColorTool(QWidget *parent)
    : QWidget(parent)
    , cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , selects_(new pcl::PointCloud<pcl::PointXYZRGB>)
    , app_path_(QApplication::applicationDirPath())
    , select_flag_(SelectFlag::NEW)
{
    ui.setupUi(this);
    ConnectUi();
    ui.view->Viewer()->registerAreaPickingCallback(&AreaPickingCallback, this);
    ui.view->BindCloud(cloud_, selects_);
    ui.lineEdit_simple->setValidator(new QDoubleValidator(ui.lineEdit_simple));
}

void QColorTool::ConnectUi()
{
    connect(ui.toolButton_open, &QToolButton::clicked, this, &QColorTool::on_Load_las);
    connect(ui.toolButton_save, &QToolButton::clicked, this, &QColorTool::on_save_las);
    connect(ui.toolButton_select_new, &QToolButton::clicked, this, &QColorTool::on_select_new);
    connect(ui.toolButton_select_union, &QToolButton::clicked, this, &QColorTool::on_select_union);
    connect(ui.toolButton_select_inter, &QToolButton::clicked, this, &QColorTool::on_select_inter);
    connect(ui.toolButton_select_diff, &QToolButton::clicked, this, &QColorTool::on_select_diff);
    connect(ui.toolButton_simple, &QToolButton::clicked, this, &QColorTool::on_simple);
    connect(ui.toolButton_color, &QToolButton::clicked, this, &QColorTool::on_color);
    connect(ui.toolButton_delete, &QToolButton::clicked, this, &QColorTool::on_delete);
//     connect(this, SIGNAL(AreaPickingSignal(boost::shared_ptr<std::vector<int>>)), 
//         this, SLOT(AreaPickingSlot(boost::shared_ptr<std::vector<int>>)), Qt::DirectConnection);
}

void QColorTool::on_Load_las()
{
    CleanSelects();
    QString path = QFileDialog::getOpenFileName(this,
        QStringLiteral("打开点云"), app_path_, QStringLiteral("Point Cloud(*.las)"));
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(path.toLocal8Bit());
    if (!lasreadopener.active())
    {
        return;
    }
    LASreader* lasreader = lasreadopener.open();
    cloud_->resize(lasreader->header.number_of_point_records);
    size_t step = 0;
    pcl::PointXYZRGB *pt;
    while (lasreader->read_point())
    {
        pt = &cloud_->at(step);
        pt->x = lasreader->point.get_x();
        pt->y = lasreader->point.get_y();
        pt->z = lasreader->point.get_z();

        pt->r = lasreader->point.get_R() / (256);
        pt->g = lasreader->point.get_G() / (256);
        pt->b = lasreader->point.get_B() / (256);
        if (pt->r == 0 && pt->g == 0 && pt->b == 0)  // 因为cgal需要两种以上不为黑的的颜色，才认为las含有颜色信息。  所以不要默认色设置成黑色。
        {
            pt->r = 1;
            pt->g = 1;
            pt->b = 1;
        }
        ++step;
    }
    lasreader->close();
    delete lasreader;
    ui.view->UpdateView();
    ui.view->Viewer()->resetCamera();
    setWindowTitle(QStringLiteral("QColorTool -- ") + path);
}

void QColorTool::on_save_las()
{
    QString path = QFileDialog::getSaveFileName(this,
        QStringLiteral("保存点云"), QStringLiteral(""), QStringLiteral("*.las")); //选择路径
    if (path.isEmpty())
        return;

    LASheader lasheader;
    lasheader.x_offset = 0;
    lasheader.y_offset = 0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 3;	//1是xyzrgb		,颜色设置不上，未找到原因暂时屏蔽掉
    lasheader.point_data_record_length = 34;
    
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(path.toLocal8Bit().data());
    if (!laswriteopener.active())
    {
        return;
    }

    // init point 

    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

    // open laswriter

    LASwriter* laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0)
    {
        fprintf(stderr, "ERROR: could not open laswriter\n");
        return;
    }
    for (auto it = cloud_->begin(); it != cloud_->end(); ++it)
    {
        laspoint.set_x(it->x);
        laspoint.set_y(it->y);
        laspoint.set_z(it->z);
        laspoint.set_R(it->r * 256);
        laspoint.set_G(it->g * 256);
        laspoint.set_B(it->b * 256);
        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);
    }

    // update the header
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
}

void QColorTool::on_select_new()
{
    select_flag_ = SelectFlag::NEW;
}

void QColorTool::on_select_union()
{
    select_flag_ = SelectFlag::UNION;
}

void QColorTool::on_select_inter()
{
    select_flag_ = SelectFlag::INTER;
}

void QColorTool::on_select_diff()
{
    select_flag_ = SelectFlag::DIFF;
}

void QColorTool::AllowOper(bool allow)
{
    ui.view->AllowOperSlot(allow);
}

void QColorTool::AreaPickingCallback(const pcl::visualization::AreaPickingEvent &event, void *args)
{
    if (!operaing_.try_lock())
    {
        return;
    }
    QColorTool *colorTool = (QColorTool*)args;
    colorTool->AllowOper(false);
    std::vector<int> indices;
    event.getPointsIndices(indices);
    std::set<int> indices_set(indices.begin(), indices.end());
    if (*indices_set.begin() >= 0 && *indices_set.end() < colorTool->cloud_->size())
    {
        colorTool->AreaPickingSlot(indices_set);
    }

    QApplication::processEvents();
    colorTool->AllowOper(true);
    operaing_.unlock();
}

void QColorTool::AreaPickingSlot(std::set<int> &indices)
{
    if (!indices.size()) // 选择集为空的时候，老容易崩溃，不知道什么内部原因，先绕行 ，但是还崩溃， 蹦在cgal的代码里的
    {
        CleanSelects();
        return;
    }
        
    switch (select_flag_)
    {
    case SelectFlag::NEW:
    {
        CleanSelects();
        for (auto pt_index : indices)
        {
            selects_index_.insert(pt_index);
        }
        break;
    }

    case SelectFlag::UNION:
    {
        for (auto pt_index : indices)
        {
            selects_index_.insert(pt_index);
        }
        break;
    }

    case SelectFlag::INTER:
    {
        std::set<int, std::greater<int>>::iterator pre_select_it;
        std::set<int> indices_set(indices.begin(), indices.end());
        for (auto it = selects_index_.begin(); it != selects_index_.end(); )
        {
            if (indices_set.find(*it) == indices_set.end())
            {
                selects_index_.erase(it++);  // set删除元素维护迭代器必须是这种写法，请注意
            }
            else
            {
                ++it;
            }
        }
        break;
    }

    case SelectFlag::DIFF:
    {
        std::set<int> indices_set(indices.begin(), indices.end());
        for (auto it = selects_index_.begin(); it != selects_index_.end(); )
        {
            if (indices_set.find(*it) != indices_set.end())
            {
                selects_index_.erase(it++);  // set删除元素维护迭代器必须是这种写法，请注意
            }
            else
            {
                ++it;
            }
        }
        break;
    }

    default:
        break;
    }

    int step = 0;
    selects_->resize(selects_index_.size());
    for (auto pt_index : selects_index_)
    {
        selects_->at(step) = cloud_->at(pt_index);
        selects_->at(step).r = 0;
        selects_->at(step).g = 255;
        selects_->at(step).b = 0;
        ++step;
    }

    ui.view->UpdateView();
}

void QColorTool::CleanSelects()
{
    selects_->clear();
    selects_index_.clear();
}

void QColorTool::on_simple()
{
    if (!operaing_.try_lock())
    {
        return;
    }
    
    double leaf = ui.lineEdit_simple->text().toDouble();
    if (leaf == 0)
    {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("网格尺寸不能为空"));
        ui.lineEdit_simple->setFocus();
        operaing_.unlock();
        return;
    }
    AllowOper(false);


    //均匀采样点云并提取关键点      体素下采样，重心代替
//     pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
//     uniform_sampling.setInputCloud(cloud_);  //输入点云
//     uniform_sampling.setRadiusSearch(leaf);   //设置半径 model_ss_初值是0.01可以通过agv修改
//     uniform_sampling.filter(*cloud_);   //滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(cloud_);
    grid.filter(*cloud_);


    CleanSelects();
    ui.view->UpdateView(true);
    QApplication::processEvents();
    AllowOper(true);
    operaing_.unlock();
    
}

void QColorTool::on_color()
{


    if (!operaing_.try_lock())
    {
        return;
    }
        

    QColor color = QColorDialog::getColor(Qt::black);
     if (!color.isValid())
     {
         operaing_.unlock();
         return;
     }
     AllowOper(false);
     for (auto it = selects_index_.begin(); it != selects_index_.end(); ++it)
     {
         cloud_->at(*it).r = color.red();
         cloud_->at(*it).g = color.green();
         cloud_->at(*it).b = color.blue();
     }

     CleanSelects();
     ui.view->UpdateView();
     QApplication::processEvents();
     AllowOper(true);
     operaing_.unlock();

}

void QColorTool::on_delete()
{
    if (!operaing_.try_lock())
    {
        return;
    }
    AllowOper(false);
    pcl::PointIndicesPtr indices(new pcl::PointIndices);
    indices->indices.insert(indices->indices.begin(), selects_index_.begin(), selects_index_.end());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud_);

    CleanSelects();

    ui.view->UpdateView(true);
    QApplication::processEvents();
     AllowOper(true);
    operaing_.unlock();
}

void QColorTool::keyPressEvent(QKeyEvent  *event)
{
    if (event->key() == Qt::Key_D)
    {
        on_delete();
    }
    QWidget::keyPressEvent(event);
}