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
#include "setting.hpp"
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

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

 void ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
 {
     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
     extract.setInputCloud(cloud);
     extract.setIndices(inices);
     extract.filter(*out_cloud);
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
     std::vector<std::tuple<pcl::PointXYZRGB, pcl::PointXYZRGB, double, unsigned int>> all_crash_point_distance;
     std::set<int> redLine;
 #pragma omp parallel for
     for (int i = 0; i < lineClusters_.size(); ++i)
     {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr line(new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr serachLine(new pcl::PointCloud<pcl::PointXYZRGB>);
         ExtractCloud(src_cloud_, boost::make_shared<pcl::PointIndices>(lineClusters_[i]), line);
 
         pcl::KdTreeFLANN<pcl::PointXYZRGB> groundkdtree;
         groundkdtree.setInputCloud(ground);
         groundkdtree.setSortedResults(true);
         pcl::KdTreeFLANN<pcl::PointXYZRGB> objectkdtree;
         objectkdtree.setInputCloud(groundObject);
         objectkdtree.setSortedResults(true);
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
         std::vector<std::tuple<pcl::PointXYZRGB, pcl::PointXYZRGB, double, unsigned int>> crash_point_distance;
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint(new  pcl::PointCloud<pcl::PointXYZRGB>);
         for (int j = 0; j < serachLine->size(); ++j)
         {
             serachNum = groundkdtree.radiusSearch(serachLine->at(j), K, groundindices, groundsqr_distances) + objectkdtree.radiusSearch(serachLine->at(j), K, objectindices, objectsqr_distances);
             if (serachNum)
             {
                 linekdtree.radiusSearch(serachLine->at(j), leafSize * 2, lineindices, line_distances);
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

                 // 记录最近碰撞点， 判断隐患类型
                 pcl::PointXYZRGB crash_linept, crash_otherpt;
                 int crash_type = 0;
                 int min_dis = std::numeric_limits<int>::max();
                 if (groundsqr_distances.size())
                 {
                     crash_type = crash_type | (int)CrashType::Ground  ;
                     min_dis = groundsqr_distances[0];
                     crash_otherpt = ground->at(groundindices[0]);
                 }
                 if (objectsqr_distances.size() && objectsqr_distances[0] < min_dis)
                 {
                     crash_type = crash_type| (int)CrashType::Other ;
                     min_dis = objectsqr_distances[0];
                     crash_otherpt = groundObject->at(objectindices[0]);
                 }
                 crash_linept = serachLine->at(j);
                 crash_point_distance.push_back(make_tuple(crash_linept, crash_otherpt, pct::Distance3d(crash_linept, crash_otherpt), crash_type));

                 // 保存碰撞点
                 crashPoint->push_back(crash_linept);
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
                 all_crash_point_distance.insert(all_crash_point_distance.end(), crash_point_distance.begin(), crash_point_distance.end());
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


     // 合并碰撞点, 计算碰撞结果
     pcl::KdTreeFLANN<pcl::PointXYZRGB> groundkdtree;
     groundkdtree.setInputCloud(ground);
     std::vector<int> groundindices;
     std::vector<float> groundsqr_distances;

     std::vector<pcl::PointIndices> cluster_indices;
     pct::ouShiFenGe(allCrashPoint, cluster_indices, gap);
     /* QString fmt = QStringLiteral("碰撞位置：x\ty\tz\t范围\n");*/ //vtk不支持中文和\t,网上的几种方法试了，是针对vtk6.0左右的版本，7.0无方法
     Eigen::Vector4f min_pt, max_pt;
     for (int i = 0; i < cluster_indices.size(); ++i)
     {
         pcl::getMinMax3D(*allCrashPoint, cluster_indices[i], min_pt, max_pt);
         CollisionBall c;
         c.min.x = min_pt.x();
         c.min.y = min_pt.y();
         c.min.z = min_pt.z();
         c.max.x = max_pt.x();
         c.max.y = max_pt.y();
         c.max.z = max_pt.z();
         c.cen.x = (min_pt.x() + max_pt.x()) / 2;
         c.cen.y = (min_pt.y() + max_pt.y()) / 2;
         c.cen.z = (min_pt.z() + max_pt.z()) / 2;
         c.nearst.dis = std::numeric_limits<int>::max();

         int nearst_index = -1;
         std::tuple<pcl::PointXYZRGB, pcl::PointXYZRGB, double, unsigned int> traverse_tuple;
         for (int j = 0; j < cluster_indices[i].indices.size(); ++j)
         {
             traverse_tuple = all_crash_point_distance[cluster_indices[i].indices[j]];
             if (std::get<2>(traverse_tuple) < c.nearst.dis)
             {
                 c.nearst.dis = std::get<2>(traverse_tuple);
                 nearst_index = cluster_indices[i].indices[j];
             }
         }
        
         c.nearst.linept = std::get<0>(all_crash_point_distance[nearst_index]);
         c.nearst.otherpt = std::get<1>(all_crash_point_distance[nearst_index]);
         c.nearst.dis = std::get<2>(all_crash_point_distance[nearst_index]);
         c.nearst.subVec = vec(c.nearst.linept.x, c.nearst.linept.y, c.nearst.linept.z) - vec(c.nearst.otherpt.x, c.nearst.otherpt.y, c.nearst.otherpt.z);
         c.crashtype = std::get<3>(all_crash_point_distance[nearst_index]);
         c.overtoplimit = K / c.nearst.dis;
         groundindices.resize(1);   groundsqr_distances.resize(1);
         groundkdtree.nearestKSearch(c.nearst.linept, 1, groundindices, groundsqr_distances);
         c.ground_distance = std::sqrt(groundsqr_distances[0]);
         c.radiu = std::max(K, (double)sqrt(pow(min_pt.x() - max_pt.x(), 2) + pow(min_pt.y() - max_pt.y(), 2) + pow(min_pt.z() - max_pt.z(), 2)) / 2);
         c.id = (QStringLiteral("ball") + QString::number(i)).toLocal8Bit().data();
         c.description = QString().sprintf("%.1f  %.1f  %.1f  %.1f\n", c.cen.x, c.cen.y, c.cen.z, c.radiu).toStdString();
         balls_.push_back(c);
         std::cout.setf(ios::fixed, ios::floatfield);
         std::cout << fixed << setprecision(6);
         std::cout << c << endl;
     }
     std::cout << "balls_数量" << balls_.size() << std::endl;
}

void StringReplace(string &strBase, string strSrc, string strDes)
{
    string::size_type pos = 0;
    string::size_type srcLen = strSrc.size();
    string::size_type desLen = strDes.size();
    pos = strBase.find(strSrc, pos);
    while ((pos != string::npos))
    {
        strBase.replace(pos, srcLen, strDes);
        pos = strBase.find(strSrc, (pos + desLen));
    }
}

template <typename T>
void streamCat(std::stringstream& ss, T num)
{
    ss.clear();
    ss.str("");
    ss << num;
}
void DangerousDistanceCheck::showNearCheck()
{
    const pct::Setting &setting = pct::Setting::ins();
    
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
    view->saveScreenshot(setting.outputdir + "\\output_xy.png");

    std::stringstream sstr;
    sstr.setf(ios::fixed, ios::floatfield);
    sstr << fixed << setprecision(6);
    sstr << dangerousDistance_;
    boost::property_tree::ptree pt;
    pt.put("距离要求.地面", sstr.str());
    pt.put("距离要求.植被", sstr.str());
    pt.put("图例颜色.地面", setting.cls_strcolor(ground_str));
    pt.put("图例颜色.铁塔", setting.cls_strcolor(tower_str));
    pt.put("图例颜色.电力线", setting.cls_strcolor(power_line_str));
    pt.put("图例颜色.植被", setting.cls_strcolor(veget_str));
    pt.put("xy平面图路径", /*setting.outputdir +*/ "./output_xy.png");
    boost::property_tree::ptree errpt_array;
     for (int i = 0; i < balls_.size(); ++i)
     {
         boost::property_tree::ptree errpt_child;
         streamCat<int>(sstr, i);
         errpt_child.put("序号", sstr.str());
         streamCat<std::string>(sstr, "-");
         errpt_child.put("塔杆区间", sstr.str());
         sstr.clear();
         sstr.str("");
         sstr << balls_[i].cen.x << "<br/>" << balls_[i].cen.y << "<br/>" << balls_[i].cen.z;
         std::cout << balls_[i].cen.x << "<br/>" << balls_[i].cen.y << "<br/>" << balls_[i].cen.z;
         errpt_child.put("隐患坐标", sstr.str());
         streamCat<double>(sstr, balls_[i].radiu);
         errpt_child.put("隐患半径", sstr.str());
         if (balls_[i].crashtype & Ground && balls_[i].crashtype &Other)
             streamCat<std::string>(sstr, "地面&植被");
         else if (balls_[i].crashtype & Ground)
             streamCat<std::string>(sstr, "地面");
         else if (balls_[i].crashtype & Other)
             streamCat<std::string>(sstr, "植被");
         else
             streamCat<std::string>(sstr, "");
         errpt_child.put("隐患类型", sstr.str());
         streamCat<double>(sstr, balls_[i].nearst.dis);
         errpt_child.put("隐患距离", sstr.str());
         streamCat<double>(sstr, pct::Distance2d(balls_[i].nearst.linept, balls_[i].nearst.otherpt));
         errpt_child.put("水平距离", sstr.str());
         streamCat<double>(sstr, std::abs(balls_[i].nearst.linept.z-balls_[i].nearst.otherpt.z));
         errpt_child.put("垂直距离", sstr.str());
         streamCat<double>(sstr, balls_[i].ground_distance);
         errpt_child.put("对地距离", sstr.str());
         streamCat<double>(sstr, balls_[i].overtoplimit);
         errpt_child.put("超限率", sstr.str());
         errpt_array.push_back(std::make_pair("", errpt_child));
     }
     pt.put_child("隐患列表", errpt_array);
     std::string json_path = setting.outputdir + "/" + pct::ExtractExeName(setting.inputfile) + "检测结果.json";
     std::ofstream ofs(json_path, fstream::out);
     boost::property_tree::write_json(ofs, pt);
     ofs.close();

      // boost生成的json自动加了转义字符，  我们需要把它去掉。
      std::string json;
      std::ifstream ifs(json_path, std::ifstream::in); 
      if (ifs.is_open())
      {
          std::stringstream buffer;
          buffer << ifs.rdbuf();
          json = buffer.str();
          ifs.close();
      }
 
 
      ofs.clear();
      ofs.open(json_path, std::ios::out | std::ios::binary);
      if (ofs.is_open())
      {
          StringReplace(json, "\\/", "/");
          StringReplace(json, "\\\\", "\\");
          json = pct::to_utf8(pct::String2WString(json));
          ofs << json;
          ofs.close();
      }
}

std::ostream& operator << (std::ostream& output, DangerousDistanceCheck::CollisionBall& c)
{
    output << "\ncen:" << c.cen << "\nmin:" << c.min << "\nmax:" << c.max
        << "\nradiu:" << c.radiu << "\ncrashtype:" << c.crashtype << "\novertoplimit:" << c.overtoplimit
        << "\nground_distance:" << c.ground_distance << "\nid:" << c.id << "\ndescription:" << c.description << "\nearst.linept:" << c.nearst.linept
        << "\nearst.otherpt:" << c.nearst.otherpt << "\nearst.subVec:" << c.nearst.subVec << "\nearst.dis:" << c.nearst.dis;
    return output;
}
