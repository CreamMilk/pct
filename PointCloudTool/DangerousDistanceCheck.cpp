#include "DangerousDistanceCheck.h"
 #include <set>
 #include <windows.h>
 #include <pcl/visualization/cloud_viewer.h>   //类cloud_viewer头文件申明
 #include <pcl/filters/extract_indices.h>
 #include "commonfunctions.h"
 #include "setting.hpp"
 #include <pcl/kdtree/kdtree_flann.h>
 #include <pcl/filters/voxel_grid.h>
#include <pcl/common/impl/common.hpp>
 #include "mydef.h"

 DangerousDistanceCheck::DangerousDistanceCheck()
 :dangerousDistance_(10)
 {
 
 }
 
 
 DangerousDistanceCheck::~DangerousDistanceCheck()
 {
 }

 void DangerousDistanceCheck::setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud
     , pcl::PointIndicesPtr ground_indices
     , std::vector <pcl::PointIndices> &otherClusters
     , std::vector <pcl::PointIndices> &lineClusters
     , std::vector <pcl::PointIndices> &towerClusters
     , double dangerousDistance)
 {
     src_cloud_ = src_cloud;
     otherClusters_ = otherClusters;
     lineClusters_ = lineClusters;
     towerClusters_ = towerClusters;
     ground_indices_ = ground_indices;
     dangerousDistance_ = dangerousDistance;
 }

//  void DangerousDistanceCheck::TooNearCheck()
//  {
//      unsigned int cur, sta;
//      sta = GetTickCount();
//  
//      RemoveViewBalls();
//      balls_.clear();
//      // 判断每根线与地物和地面范围K内是否有交集，计算出电力线与地面地物最大的交集
//      // 如果计算量过大 可以考虑先降采样
//      int gap = 10;
//      double K = dangerousDistance_;
//      double leafSize = 1.5;
//      unsigned char labr = 255, labb = 0, labg = 0;
//  
//      std::vector<int> sectionBeginSet;
//      int stepindex = 0;
//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundObject(new pcl::PointCloud<pcl::PointXYZRGB>);
//  
//      for (int i = 0; i < groundObjects_.size(); ++i)
//      {
//          sectionBeginSet.push_back(stepindex);
//          groundObject->insert(groundObject->end(), groundObjects_[i]->begin(), groundObjects_[i]->end());
//          stepindex += groundObjects_[i]->size();
//      }
//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCrashPoint(new pcl::PointCloud<pcl::PointXYZRGB>);
//  
//      std::set<int> redLine;
//  #pragma omp parallel for
//      for (int i = 0; i < lines_.size(); ++i)
//      {
//          pcl::KdTreeFLANN<pcl::PointXYZRGB> groundkdtree;
//          groundkdtree.setInputCloud(ground_);
//          pcl::KdTreeFLANN<pcl::PointXYZRGB> objectkdtree;
//          objectkdtree.setInputCloud(groundObject);
//          pcl::KdTreeFLANN<pcl::PointXYZRGB> linekdtree;
//          linekdtree.setInputCloud(lines_[i]);
//  
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr line = lines_[i];
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr serachLine(pcl::PointCloud<pcl::PointXYZRGB>);
//  
//          pcl::VoxelGrid<pcl::PointXYZRGB> grid;
//          grid.setLeafSize(leafSize, leafSize, leafSize);
//          grid.setInputCloud(line);
//          grid.filter(*serachLine);
//  
//          int serachNum = 0;
//          std::vector<int> groundindices;
//          std::vector<float> groundsqr_distances;
//          std::vector<int> objectindices;
//          std::vector<float> objectsqr_distances;
//          std::vector<int> lineindices;
//          std::vector<float> line_distances;
//  
//  
//          std::set<int> line_groundindices;
//          std::set<int> line_objectindices;
//  
//          // 得到此根线的最大相交范围点
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint(new  pcl::PointCloud<pcl::PointXYZRGB>);
//          for (int j = 0; j < serachLine->size(); ++j)
//          {
//              serachNum = groundkdtree.radiusSearch(serachLine->at(j), K, groundindices, groundsqr_distances) + objectkdtree.radiusSearch(serachLine->at(j), K, objectindices, objectsqr_distances);
//              if (serachNum)
//              {
//                  linekdtree.radiusSearch(serachLine->at(j), leafSize * 2, lineindices, line_distances);;
//                  for (int k = 0; k < lineindices.size(); ++k)
//                  {
//  #pragma omp critical 
//                      if (redLine.find(lineindices[k]) == redLine.end())
//                      {
//                          redLine.insert(lineindices[k]);
//                          line->at(lineindices[k]).r = 255;
//                          line->at(lineindices[k]).g = 0;
//                          line->at(lineindices[k]).b = 0;
//                      }
//                  }
//                  crashPoint->push_back(serachLine->at(j));
//                  for (int k = 0; k < groundindices.size(); ++k)
//                  {
//                      line_groundindices.insert(groundindices[k]);
//                  }
//                  for (int k = 0; k < objectindices.size(); ++k)
//                  {
//                      line_objectindices.insert(objectindices[k]);
//                  }
//              }
//          }
//  
//          // 如果相交了
//          if (crashPoint->size())
//          {
//              allCrashPoint->insert(allCrashPoint->end(), crashPoint->begin(), crashPoint->end());
//              // 地面和地物碰撞标记
//  #pragma omp critical  
//              for (std::set<int>::iterator it = line_groundindices.begin(); it != line_groundindices.end(); ++it)
//              {
//                  ground_->at(*it).r = 0;
//                  ground_->at(*it).g = 0;
//                  ground_->at(*it).b = 255;
//              }
//  
//              int session = 0;
//              int sessionnum = 0;
//  #pragma omp critical 
//              for (std::set<int>::iterator it = line_objectindices.begin(); it != line_objectindices.end(); ++it)
//              {
//                  // 遍历索引数组，判断maxobjectindices[k]是第几个地物数组的内容
//                  // 之所以可以这样做是因为maxobjectindices是一个顺序排列的特殊的索引，groundObject是一个特殊的点云
//                  for (int n = sectionBeginSet.size() - 1; n >= 0; --n)
//                  {
//                      if (sectionBeginSet[n] <= *it)
//                      {
//                          session = n;
//                          sessionnum = sectionBeginSet[n];
//                          break;
//                      }
//                  }
//                  pcl::PointXYZRGB &pt = groundObjects_[session]->at(*it - sessionnum);
//                  {
//                      pt.r = 0;
//                      pt.g = 255;
//                      pt.b = 0;
//                  }
//              }
//          }
//      }
//  
//      LabelCrashLine(allCrashPoint, K, gap);
//      UpdateShowClass("碰撞球");
//      ShowTooNearCheck();
//  }
 
 void ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
 {
     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
     extract.setInputCloud(cloud);
     extract.setIndices(inices);
     extract.filter(*out_cloud);
 }

 void DangerousDistanceCheck::LabelCrashLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint, double K, double gap)
 {
     std::vector<pcl::PointIndices> cluster_indices;
     pct::ouShiFenGe(crashPoint, cluster_indices, gap);
     /* QString fmt = QStringLiteral("碰撞位置：x\ty\tz\t范围\n");*/ //vtk不支持中文和\t,网上的几种方法试了，是针对vtk6.0左右的版本，7.0无方法

     Eigen::Vector4f min_pt, max_pt;
     for (int i = 0; i < cluster_indices.size(); ++i)
     {
         pcl::getMinMax3D(*crashPoint, cluster_indices[i], min_pt, max_pt);
         CollisionBall c;
         c.cen.x = (min_pt.x() + max_pt.x()) / 2;
         c.cen.y = (min_pt.y() + max_pt.y()) / 2;
         c.cen.z = (min_pt.z() + max_pt.z()) / 2;
         c.radiu = std::max(K, (double)sqrt(pow(min_pt.x() - max_pt.x(), 2) + pow(min_pt.y() - max_pt.y(), 2) + pow(min_pt.z() - max_pt.z(), 2)) / 2);
         c.id = (QStringLiteral("ball") + QString::number(i)).toLocal8Bit().data();
         c.description = QString().sprintf("%.1f  %.1f  %.1f  %.1f\n", c.cen.x, c.cen.y, c.cen.z, c.radiu).toStdString();
         // fmt += QString().sprintf("%.1f  %.1f  %.1f  %.1f\n", c.cen.x, c.cen.y, c.cen.z, c.radiu);

         balls_.push_back(c);
     }
 }

void DangerousDistanceCheck::TooNearCheck()
{
     unsigned int cur, sta;
     sta = GetTickCount();
 
     balls_.clear();
     // 判断每根线与地物和地面范围K内是否有交集，计算出电力线与地面地物最大的交集
     // 如果计算量过大 可以考虑先降采样
     int gap = 10;
     double K = dangerousDistance_;
     double leafSize = 1.5;
 
     const pct::Setting & setting = pct::Setting::ins();
     unsigned int color = setting.cls_intcolor(crash_line_str);
     unsigned char crash_line_r = *(((unsigned char *)&color) + 2);
     unsigned char crash_line_g = *(((unsigned char *)&color) + 1);
     unsigned char crash_line_b = *(((unsigned char *)&color) + 0);
     color = setting.cls_intcolor(crash_other_str);
     unsigned char crash_other_r = *(((unsigned char *)&color) + 2);
     unsigned char crash_other_g = *(((unsigned char *)&color) + 1);
     unsigned char crash_other_b = *(((unsigned char *)&color) + 0);
 
     std::vector<int> sectionBeginSet;
     int stepindex = 0;
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundObject(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
     ExtractCloud(src_cloud_, ground_indices_, ground);
     std::cout << "ground数量" << ground->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;

     for (int i = 0; i < otherClusters_.size(); ++i)
     {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_object(new pcl::PointCloud<pcl::PointXYZRGB>);
         ExtractCloud(src_cloud_, boost::make_shared<pcl::PointIndices>(otherClusters_[i]), tmp_object);
         sectionBeginSet.push_back(stepindex);
         groundObject->insert(groundObject->end(), tmp_object->begin(), tmp_object->end());
         stepindex += otherClusters_[i].indices.size();
     }
     std::cout << "groundObject数量" << groundObject->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCrashPoint(new pcl::PointCloud<pcl::PointXYZRGB>);
 
     std::set<int> redLine;
 #pragma omp parallel for
     for (int i = 0; i < lineClusters_.size(); ++i)
     {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr line(new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr serachLine(new pcl::PointCloud<pcl::PointXYZRGB>);
         ExtractCloud(src_cloud_, boost::make_shared<pcl::PointIndices>(lineClusters_[i]), line);
 
         pcl::KdTreeFLANN<pcl::PointXYZRGB> groundkdtree;
         groundkdtree.setInputCloud(ground);
         pcl::KdTreeFLANN<pcl::PointXYZRGB> objectkdtree;
         objectkdtree.setInputCloud(groundObject);
         pcl::KdTreeFLANN<pcl::PointXYZRGB> linekdtree;
         linekdtree.setInputCloud(line);
 
         pcl::VoxelGrid<pcl::PointXYZRGB> grid;
         grid.setLeafSize(leafSize, leafSize, leafSize);
         grid.setInputCloud(line);
         grid.filter(*serachLine);
 
         int serachNum = 0;
         std::vector<int> groundindices;
         std::vector<float> groundsqr_distances;
         std::vector<int> objectindices;
         std::vector<float> objectsqr_distances;
         std::vector<int> lineindices;
         std::vector<float> line_distances;
 
 
         std::set<int> line_groundindices;
         std::set<int> line_objectindices;
 
         // 得到此根线的最大相交范围点
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint(new  pcl::PointCloud<pcl::PointXYZRGB>);
         for (int j = 0; j < serachLine->size(); ++j)
         {
             serachNum = groundkdtree.radiusSearch(serachLine->at(j), K, groundindices, groundsqr_distances) + objectkdtree.radiusSearch(serachLine->at(j), K, objectindices, objectsqr_distances);
             if (serachNum)
             {
                 linekdtree.radiusSearch(serachLine->at(j), leafSize * 2, lineindices, line_distances);;
                 for (int k = 0; k < lineindices.size(); ++k)
                 {
 #pragma omp critical 
                     if (redLine.find(lineindices[k]) == redLine.end())
                     {
                         redLine.insert(lineindices[k]);
                         src_cloud_->at(lineClusters_[i].indices[lineindices[k]]).r = crash_line_r;
                         src_cloud_->at(lineClusters_[i].indices[lineindices[k]]).g = crash_line_g;
                         src_cloud_->at(lineClusters_[i].indices[lineindices[k]]).b = crash_line_b;
                     }
                 }
                 crashPoint->push_back(serachLine->at(j));
                 for (int k = 0; k < groundindices.size(); ++k)
                 {
                     line_groundindices.insert(groundindices[k]);
                 }
                 for (int k = 0; k < objectindices.size(); ++k)
                 {
                     line_objectindices.insert(objectindices[k]);
                 }
             }
         }
 
         // 如果相交了
         if (crashPoint->size())
         {
             // 地面和地物碰撞标记
 #pragma omp critical  
             {
                 allCrashPoint->insert(allCrashPoint->end(), crashPoint->begin(), crashPoint->end());
                 for (std::set<int>::iterator it = line_groundindices.begin(); it != line_groundindices.end(); ++it)
                 {
                     pcl::PointXYZRGB &tmp_pt = src_cloud_->at(ground_indices_->indices[*it]);
                     tmp_pt.r = crash_other_r;
                     tmp_pt.g = crash_other_g;
                     tmp_pt.b = crash_other_b;
                 }
             }

 
             int session = 0;
             int sessionnum = 0;
 #pragma omp critical 
             for (std::set<int>::iterator it = line_objectindices.begin(); it != line_objectindices.end(); ++it)
             {
                 // 遍历索引数组，判断maxobjectindices[k]是第几个地物数组的内容
                 // 之所以可以这样做是因为maxobjectindices是一个顺序排列的特殊的索引，groundObject是一个特殊的点云
                 for (int n = sectionBeginSet.size() - 1; n >= 0; --n)
                 {
                     if (sectionBeginSet[n] <= *it)
                     {
                         session = n;
                         sessionnum = sectionBeginSet[n];
                         break;
                     }
                 }
                 //pcl::PointXYZRGB &pt = groundObjects_[session]->at(*it - sessionnum);
                 pcl::PointXYZRGB &pt = src_cloud_->at(otherClusters_[session].indices[*it - sessionnum]);
                 {
                     pt.r = crash_other_r;
                     pt.g = crash_other_g;
                     pt.b = crash_other_b;
                 }
             }
         }
     }
     std::cout << "allCrashPoint数量" << allCrashPoint->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;
     LabelCrashLine(allCrashPoint, dangerousDistance_, gap);
     std::cout << "balls_数量" << balls_.size() << std::endl;
}

void DangerousDistanceCheck::showNearCheck()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test"));
    view->setBackgroundColor(1, 1, 1);

    view->addPointCloud<pcl::PointXYZRGB>(src_cloud_, "cloud");      // no need to add the handler, we use a random handler by default
    QString fmt = QStringLiteral("Crash Position:\nx  y  z  Radius\n");
    for (int i = 0; i < balls_.size(); ++i)
    {
        fmt += QString::fromStdString(balls_[i].description);
        view->addSphere(balls_[i].cen, balls_[i].radiu, 1, 1, 0, balls_[i].id);
        view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, balls_[i].id);
    }

    int lineHeight = 14;
    double vp[4];
    vtkRenderer *render = view->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    render->GetViewport(vp);
    render->NormalizedDisplayToDisplay(vp[0], vp[1]);
    render->NormalizedDisplayToDisplay(vp[2], vp[3]);
    double dy = vp[3] - vp[1];
    if (!view->addText(fmt.toUtf8().data(), lineHeight, max(0, (int)dy - (3 + (int)balls_.size()) * lineHeight), 0, 0, 1, "crashText"))
        view->updateText(fmt.toUtf8().data(), lineHeight, max(0, (int)dy - (3 + (int)balls_.size()) * lineHeight), 0, 0, 1, "crashText");


    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*src_cloud_, min, max);
    std::stringstream minstr, maxstr, counts;
    counts << "counts: " << src_cloud_->size();
    minstr << std::fixed << setprecision(2) << "min: " << min.x << " " << min.y << " " << min.z;
    maxstr << std::fixed << setprecision(2) << "max: " << max.x << " " << max.y << " " << max.z;
    view->addText(counts.str(), 5, 30, 1, 0, 1, std::string("counts"));
    view->addText(minstr.str(), 5, 20, 1, 0, 1, std::string("aabbBoxmin"));
    view->addText(maxstr.str(), 5, 10, 1, 0, 1, std::string("aabbBoxmax"));

    view->getRenderWindow()->Render();
    view->saveScreenshot("bbbbbbbbbbb.png");
}