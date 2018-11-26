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

#include <Geometry\OBB.h>

#include <QApplication>
#include <QDir>

 DangerousDistanceCheck::DangerousDistanceCheck()
 {
 
 }
 
 
 DangerousDistanceCheck::~DangerousDistanceCheck()
 {
 }

 void DangerousDistanceCheck::setData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud
     , pcl::PointIndicesPtr ground_indices
     , std::vector <pct::VegetInfo> &vegetClusters
     , std::vector <pct::LineInfo> &lineClusters
     , std::vector <pct::TowerInfo> &towerClusters
     )
 {
     src_cloud_ = src_cloud;
     vegetClusters_ = vegetClusters;
     lineClusters_ = lineClusters;
     towerClusters_ = towerClusters;
     ground_indices_ = ground_indices;
 }

 // 参与碰撞的点云， 
 void Check_Distanceerror_vecAndhor(pcl::PointXYZRGB pt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> &indices, std::vector<float> &distances, float hor_K, float vec_K, std::map<float, int>& dis_ind)
 {
     pcl::PointXYZRGB *tmpPt;
     for (int i = 0; i < indices.size(); ++i)
     {
         tmpPt = &cloud->at(indices[i]);
         if ((std::abs(tmpPt->z - pt.z) < vec_K) && pct::Distance2d(tmpPt->x, tmpPt->y, pt.x, pt.y) < hor_K)
         {
             dis_ind.insert(make_pair(distances[i], indices[i]));
         }
     }
 }

 // 参与碰撞的点云， 
 void Check_Distanceerror(pcl::PointXYZRGB pt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<int> &indices, std::vector<float> &distances, bool direction, float K, std::map<float, int>& dis_ind)
 {
     pcl::PointXYZRGB *tmpPt;
     for (int i = 0; i < indices.size(); ++i)
     {
         tmpPt = &cloud->at(indices[i]);
         if (direction)  // 水平
         {
             if ((std::abs(tmpPt->z - pt.z) < 0.5) && pct::Distance2d(tmpPt->x, tmpPt->y, pt.x, pt.y) < K)
             {
                 dis_ind.insert(make_pair(distances[i], indices[i]));
             }
         }
         else
         {
             if ((std::abs(tmpPt->z - pt.z) < K) && pct::Distance2d(tmpPt->x, tmpPt->y, pt.x, pt.y) <0.5)
             {
                 dis_ind.insert(make_pair(distances[i], indices[i]));
             }
         }
     }
 }

void DangerousDistanceCheck::TooNearCheck()
{
    std::cout << "TooNearCheck" << std::endl;
     unsigned int cur, sta;
     sta = GetTickCount();
 
     balls_.clear();
     // 判断每根线与地物和地面范围K内是否有交集，计算出电力线与地面地物最大的交集
     // 如果计算量过大 可以考虑先降采样
     int gap = 5;
     double leafSize = 1;
 
     const pct::Setting & setting = pct::Setting::ins();
     std::map<std::string, std::tuple<float, float>> class_disatances = setting.get_distances();
     std::cout << "get_distances" << std::endl;
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
     pct::ExtractCloud(src_cloud_, ground_indices_, ground);
     std::cout << "ground数量" << ground->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;

     for (int i = 0; i < vegetClusters_.size(); ++i)
     {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_object(new pcl::PointCloud<pcl::PointXYZRGB>);
         pct::ExtractCloud(src_cloud_, boost::make_shared<pcl::PointIndices>(vegetClusters_[i].indices), tmp_object);
         sectionBeginSet.push_back(stepindex);
         groundObject->insert(groundObject->end(), tmp_object->begin(), tmp_object->end());
         stepindex += vegetClusters_[i].indices.indices.size();
     }
     std::cout << "groundObject数量" << groundObject->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCrashPoint(new pcl::PointCloud<pcl::PointXYZRGB>);
     std::vector<std::tuple<std::string, pcl::PointXYZRGB, pcl::PointXYZRGB, double, pcl::PointXYZRGB, unsigned int>> all_crash_point_distance;
     std::set<int> redLine;
 #pragma omp parallel for
     for (int i = 0; i < lineClusters_.size(); ++i)
     {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr line(new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr serachLine(new pcl::PointCloud<pcl::PointXYZRGB>);
         pct::ExtractCloud(src_cloud_, boost::make_shared<pcl::PointIndices>(lineClusters_[i].indices), line);
 
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
         std::vector<std::tuple<std::string, pcl::PointXYZRGB, pcl::PointXYZRGB, double, pcl::PointXYZRGB, unsigned int>> crash_point_distance;
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr crashPoint(new  pcl::PointCloud<pcl::PointXYZRGB>);
         for (int j = 0; j < serachLine->size(); ++j)
         {
             float ground_horizontal_distance = std::get<0>(class_disatances["ground"]);
             float ground_vertical_distance = std::get<1>(class_disatances["ground"]);
             float ground_duijiaoxian_distance = std::sqrt(std::pow(ground_horizontal_distance,2) + std::pow(ground_vertical_distance,2));
             
             if (ground_horizontal_distance != 0 || ground_vertical_distance != 0)
             {
                 if (groundkdtree.radiusSearch(serachLine->at(j), ground_duijiaoxian_distance, groundindices, groundsqr_distances))
                 {
                     std::map<float, int> dis_ind;
                     if (ground_horizontal_distance != 0 && ground_vertical_distance != 0)
                     {
                         Check_Distanceerror_vecAndhor(serachLine->at(j), ground, groundindices, groundsqr_distances, ground_horizontal_distance, ground_vertical_distance, dis_ind);
                     }
                     else if (ground_horizontal_distance != 0)
                     {
                         Check_Distanceerror(serachLine->at(j), ground, groundindices, groundsqr_distances, true, ground_horizontal_distance, dis_ind);
                     }
                     else if (ground_vertical_distance != 0)
                     {
                         Check_Distanceerror(serachLine->at(j), ground, groundindices, groundsqr_distances, false, ground_vertical_distance, dis_ind);
                     }
                     groundindices.resize(dis_ind.size());
                     groundsqr_distances.resize(dis_ind.size());

                     int dis_ind_step = 0;
                     for (auto dis_ind_it = dis_ind.begin(); dis_ind_it != dis_ind.end(); dis_ind_it++, dis_ind_step++)
                     {
                         groundsqr_distances[dis_ind_step] = dis_ind_it->first;
                         groundindices[dis_ind_step] = dis_ind_it->second;
                     }
                 }
             }
             float object_horizontal_distance = std::get<0>(class_disatances["veget"]);
             float object_vertical_distance = std::get<1>(class_disatances["veget"]);
             float object_duijiaoxian_distance = std::sqrt(std::pow(object_horizontal_distance,2) + std::pow(object_vertical_distance,2));

             if (object_horizontal_distance != 0 || object_vertical_distance != 0)
             {
                 if (objectkdtree.radiusSearch(serachLine->at(j), object_duijiaoxian_distance, objectindices, objectsqr_distances))
                 {
                     std::map<float, int> dis_ind;
                     if (object_horizontal_distance != 0 && object_vertical_distance != 0)
                     {
                         Check_Distanceerror_vecAndhor(serachLine->at(j), groundObject, objectindices, objectsqr_distances, object_horizontal_distance, object_vertical_distance, dis_ind);
                     }
                     else if (object_horizontal_distance != 0)
                     {
                         Check_Distanceerror(serachLine->at(j), groundObject, objectindices, objectsqr_distances, true, object_horizontal_distance, dis_ind);
                     }
                     else if (object_vertical_distance != 0)
                     {
                         Check_Distanceerror(serachLine->at(j), groundObject, objectindices, objectsqr_distances, false, object_vertical_distance, dis_ind);
                     }
                     objectindices.resize(dis_ind.size());
                     objectsqr_distances.resize(dis_ind.size());

                     int dis_ind_step = 0;
                     for (auto dis_ind_it = dis_ind.begin(); dis_ind_it != dis_ind.end(); dis_ind_it++, dis_ind_step++)
                     {
                         objectsqr_distances[dis_ind_step] = dis_ind_it->first;
                         objectindices[dis_ind_step] = dis_ind_it->second;
                     }
                 }
             }
           
             serachNum = objectindices.size() + groundindices.size();


             if (serachNum)
             {
                 linekdtree.radiusSearch(serachLine->at(j), leafSize * 2.5f, lineindices, line_distances);
                 for (int k = 0; k < lineindices.size(); ++k)
                 {
 #pragma omp critical 
                     if (redLine.find(lineindices[k]) == redLine.end())
                     {
                         redLine.insert(lineindices[k]);
                         src_cloud_->at(lineClusters_[i].indices.indices[lineindices[k]]).r = crash_line_r;
                         src_cloud_->at(lineClusters_[i].indices.indices[lineindices[k]]).g = crash_line_g;
                         src_cloud_->at(lineClusters_[i].indices.indices[lineindices[k]]).b = crash_line_b;
                     }
                 }

                 // 记录最近碰撞点， 判断隐患类型
                 pcl::PointXYZRGB crash_nearst_linept, crash_nearst_pt, crash_far_pt;
                 int crash_type = 0;
                 int min_dis = std::numeric_limits<int>::max();
                 int max_dis = -std::numeric_limits<int>::max();
                 if (groundsqr_distances.size())
                 {
                     crash_type = crash_type | (int)CrashType::Ground  ;
                     min_dis = groundsqr_distances[0];
                     max_dis = groundsqr_distances[(int)groundsqr_distances.size()-1];
                     crash_nearst_pt = ground->at(groundindices[0]);
                     crash_far_pt = ground->at(groundindices[(int)groundindices.size() - 1]);
                 }
                 if (objectsqr_distances.size() )
                 {
                     crash_type = crash_type | (int)CrashType::Other;
                     if (objectsqr_distances[0] < min_dis)
                     {
                         min_dis = objectsqr_distances[0];
                         crash_nearst_pt = groundObject->at(objectindices[0]);
                     }
                     if (objectsqr_distances[(int)objectsqr_distances.size() - 1] > max_dis)
                     {
                         max_dis = objectsqr_distances[(int)objectsqr_distances.size() - 1];
                         crash_far_pt = groundObject->at(objectindices[(int)objectindices.size() - 1]);
                     }
                 }
                 crash_nearst_linept = serachLine->at(j);
                 crash_point_distance.push_back(make_tuple(lineClusters_[i].getLineNo(), crash_nearst_linept, crash_nearst_pt, pct::Distance3d(crash_nearst_linept, crash_nearst_pt), crash_far_pt, crash_type));

                 // 保存碰撞点
                 crashPoint->push_back(crash_nearst_linept);
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
         std::cout << "line_groundindices" << line_groundindices.size() << std::endl;
         std::cout << "line_objectindices" << line_objectindices.size() << std::endl;
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
                 pcl::PointXYZRGB &pt = src_cloud_->at(vegetClusters_[session].indices.indices[*it - sessionnum]);
                 {
                     pt.r = crash_other_r;
                     pt.g = crash_other_g;
                     pt.b = crash_other_b;
                 }
             }
         }
     }
     std::cout << "allCrashPoint数量" << allCrashPoint->size() << "\tsrc_cloud数量" << src_cloud_->size() << std::endl;

	 if (!allCrashPoint->size())
		 return;
     // 合并碰撞点, 计算碰撞结果
     pcl::KdTreeFLANN<pcl::PointXYZRGB> groundkdtree;
     groundkdtree.setInputCloud(ground);
     std::vector<int> groundindices;
     std::vector<float> groundsqr_distances;

     std::vector<pcl::PointIndices> cluster_indices;
     pct::ouShiFenGe(allCrashPoint, cluster_indices, gap);
     pct::mergeBalls(allCrashPoint, cluster_indices);
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

         pcl::PointXYZRGB *farst_pt = nullptr;
         float max_radiu = -std::numeric_limits<int>::max();
         float distance_float = -std::numeric_limits<int>::max();

         int nearst_index = -1;
         std::tuple<std::string, pcl::PointXYZRGB, pcl::PointXYZRGB, double, pcl::PointXYZRGB, unsigned int> traverse_tuple;
         for (int j = 0; j < cluster_indices[i].indices.size(); ++j)
         {
             traverse_tuple = all_crash_point_distance[cluster_indices[i].indices[j]];
             if (std::get<3>(traverse_tuple) < c.nearst.dis)
             {
                 c.nearst.dis = std::get<3>(traverse_tuple);
                 nearst_index = cluster_indices[i].indices[j];
             }
             distance_float = pct::Distance3d(c.cen, std::get<4>(traverse_tuple));
             if (distance_float > max_radiu)
             {
                 max_radiu = distance_float;
                 farst_pt = &std::get<4>(traverse_tuple);
             }
         }
        
         c.lineno = std::get<0>(all_crash_point_distance[nearst_index]);
         c.nearst.linept = std::get<1>(all_crash_point_distance[nearst_index]);
         c.nearst.otherpt = std::get<2>(all_crash_point_distance[nearst_index]);
         c.nearst.dis = std::get<3>(all_crash_point_distance[nearst_index]);
         c.nearst.subVec = vec(c.nearst.linept.x, c.nearst.linept.y, c.nearst.linept.z) - vec(c.nearst.otherpt.x, c.nearst.otherpt.y, c.nearst.otherpt.z);
         c.crashtype = std::get<5>(all_crash_point_distance[nearst_index]);
         if (pct::Distance2d(c.nearst.linept.x, c.nearst.linept.y, c.nearst.otherpt.x, c.nearst.otherpt.y) > std::abs(c.nearst.linept.z - c.nearst.otherpt.z))  // 水平距离危险
         {
             if ((c.crashtype & CrashType::Other) && (CrashType::Ground & c.crashtype))  // 如果是地面和地物同时碰撞，则最小的碰撞规则为最小的类型
             {
                 float k = std::min(std::get<0>(class_disatances["ground"]), std::get<0>(class_disatances["veget"]));
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
             else if (c.crashtype & CrashType::Other)  // 
             {
                 float k = std::get<0>(class_disatances["veget"]);
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
             else if (CrashType::Ground & c.crashtype)  // 
             {
                 float k = std::get<0>(class_disatances["ground"]);
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
         }
         else
         {
             if ((c.crashtype & CrashType::Other) && (CrashType::Ground & c.crashtype))  // 如果是地面和地物同时碰撞，则最小的碰撞规则为最小的类型
             {
                 float k = std::min(std::get<1>(class_disatances["ground"]), std::get<1>(class_disatances["veget"]));
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
             else if (c.crashtype & CrashType::Other)  // 
             {
                 float k = std::get<1>(class_disatances["veget"]);
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
             else if (CrashType::Ground & c.crashtype)  // 
             {
                 float k = std::get<1>(class_disatances["ground"]);
                 c.overtoplimit = (k - c.nearst.dis) / k * 100;
             }
         }
         

         // 离地距离
         groundindices.resize(1);   groundsqr_distances.resize(1);
         groundkdtree.nearestKSearch(c.nearst.linept, 1, groundindices, groundsqr_distances);
         if (groundsqr_distances.size()) 
             c.ground_distance = std::sqrt(groundsqr_distances[0]);

         // 计算包围盒半径
         c.radiu = c.GetExtraBoxRadiu(*farst_pt);
         c.id = (QStringLiteral("ball") + QString::number(i + 1)).toLocal8Bit().data();
         double x = c.cen.x;
         double y = c.cen.y;
         pct::UTMXY2LatLon(x, y);
         //CoorConv::WGS84Corr latlon;
         //CoorConv::UTMXYToLatLon(c.cen.x, c.cen.y, 50, false, latlon);
         //c.description = QString().sprintf("%.7f  %.7f  %.1f  %.1f\n", CoorConv::RadToDeg(latlon.lat), CoorConv::RadToDeg(latlon.log), c.cen.z, c.radiu).toStdString();
         c.description = QString().sprintf("%.7f  %.7f  %.1f  %.1f\n", x, y, c.cen.z, c.radiu).toStdString();
         balls_.push_back(c);
         //std::cout.setf(ios::fixed, ios::floatfield);
         //std::cout << fixed << setprecision(6);
         //std::cout << c << endl;
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

void DangerousDistanceCheck::getObbInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, OBB &obb, vec *axis_vec/*[3]*/, vec &axis_r, double midz)
{
    int ptct = src_cloud->size();
    boost::shared_ptr<vec> points(new vec[ptct], std::default_delete<vec[]>());
    for (int j = 0; j < ptct; ++j)
    {
        int &curindex = j;
        points.get()[j] = vec(src_cloud->at(curindex).x, src_cloud->at(curindex).y, /*src_cloud->at(curindex).z*/midz-1+rand()%3); // 
    }
    // 计算铁塔obb，设定铁塔最小长宽高范围为5
    
    vec diagonal;
    obb = OBB::BruteEnclosingOBB(points.get(), ptct);


    std::cout << "corners" << std::endl;
    
    memcpy(axis_vec, obb.axis, sizeof(vec) * 3);
    axis_r = obb.r;
    


     vec tempvec;
     float tempf;
     for (int i = 0; i < 3 - 1; ++i)
     {
         for (int j = i + 1; j < 3; ++j)
         {
             if (axis_r[i] < axis_r[j])
             {
                 tempvec = axis_vec[i];
                 axis_vec[i] = axis_vec[j];
                 axis_vec[j] = tempvec;
 
                 tempf = axis_r[i];
                 axis_r[i] = axis_r[j];
                 axis_r[j] = tempf;
             }
         }
     }
//     axis_vec[0].z = 0;
//     axis_vec[1].z = 0;
//     axis_vec[2].z = 0;
}

void DangerousDistanceCheck::showNearCheck()
{
    const pct::Setting &setting = pct::Setting::ins();
    unsigned int color = setting.cls_intcolor(background_str);
    unsigned char background_r = *(((unsigned char *)&color) + 2);
    unsigned char background_g = *(((unsigned char *)&color) + 1);
    unsigned char background_b = *(((unsigned char *)&color) + 0);
    color = setting.cls_intcolor(arrow_str);
    unsigned char arrow_r = *(((unsigned char *)&color) + 2);
    unsigned char arrow_g = *(((unsigned char *)&color) + 1);
    unsigned char arrow_b = *(((unsigned char *)&color) + 0);




    QString pic_dir = QString::fromLocal8Bit(setting.outputdir.c_str()) +  QStringLiteral("\\images");
    pct::DelDir(pic_dir);
    QDir().mkpath(pic_dir);
    // 准备点云信息
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test"));

    view->setBackgroundColor(background_r / 255.0, background_g / 255.0, background_b / 255.0);
    pcl::PointXYZRGB cloudmin, cloudmax;
    pcl::getMinMax3D(*src_cloud_, cloudmin, cloudmax);
    float aabb_subz = cloudmax.z - cloudmin.z;
    vec axis_vec[3];
    vec axis_r;
    OBB obb;
    getObbInfo(src_cloud_, obb, axis_vec, axis_r, (cloudmax.z + cloudmin.z) / 2);  // 根据轴长排序
    std::cout << "seevec" << vec(axis_vec[1].x*axis_r[1], axis_vec[1].y*axis_r[1], aabb_subz*0.45/*axis_vec[1].z*(aabb_subz / 2)*/) << std::endl;
    std::cout << axis_vec[0] << " " << axis_r[0] << "\n" << axis_vec[1] << " " << axis_r[1] << "\n" << axis_vec[2] << " " << axis_r[2] << " " << std::endl;
    // 添加点云
    view->addPointCloud<pcl::PointXYZRGB>(src_cloud_, "cloud");      // no need to add the handler, we use a random handler by default


    // 添加塔编号
    pcl::PointXYZRGB nopt;
    for (int i = 0; i < towerClusters_.size(); ++i)
    {
        nopt = towerClusters_[i].cen;
        nopt.z = towerClusters_[i].max.z+1;
        view->addText3D(towerClusters_[i].getNo(), nopt, 8.0, 0, 0, 1, towerClusters_[i].getNo());
    }

    // 让相机移动到合适位置，然后取相机的参数
    view->resetCamera();
    pcl::visualization::Camera camera;
    view->getCameraParameters(camera);

    // 计算相机默认方向
    vec camervec = vec(camera.focal[0], camera.focal[1], camera.focal[2]) - vec(camera.pos[0], camera.pos[1], camera.pos[2]);
    float cameralen = camervec.Length();
    
   // 计算相机最佳位置和方向
    vec seevec = vec(axis_vec[1].x*axis_r[1], axis_vec[1].y*axis_r[1], aabb_subz*0.45/*axis_vec[1].z*(aabb_subz / 2)*/).Normalized();  // 全局观看方向
    vec camerapos = obb.pos + seevec * cameralen*0.5;  // 相机位置 = 焦点 + 向量
    

    // 添加碰撞球编号
    for (int i = 0; i < balls_.size(); ++i)
    {
        std::string showtext = balls_[i].id;
        pct::StringReplace(showtext, "ball", "");
        //view->addSphere(balls_[i].cen, balls_[i].radiu, 1, 1, 0, balls_[i].id);
        //view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, balls_[i].id);
        //view->addText3D(showtext, balls_[i].cen, 8.0, 1, 0, 0, "ball" + balls_[i].id + "#");
        view->addText3D(showtext, balls_[i].nearst.linept, 8.0, arrow_r / 255.0, arrow_g / 255.0, arrow_b / 255.0, "ball" + balls_[i].id + "#");
        view->addArrow(balls_[i].nearst.linept, balls_[i].nearst.otherpt, arrow_r / 255.0, arrow_g / 255.0, arrow_b / 255.0, balls_[i].id);
        view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, balls_[i].id);
        view->addSphere(balls_[i].nearst.linept, 1, 1, 0, 0, balls_[i].id+"sphere");
        
        // 设置相机焦点（碰撞球中心点）， 设置相机位置（）
        //vec pos(balls_[i].cen.x, balls_[i].cen.y, balls_[i].cen.z);
        vec pos(balls_[i].nearst.linept.x, balls_[i].nearst.linept.y, balls_[i].nearst.linept.z);
        vec movepos = pos + seevec * std::max(balls_[i].nearst.dis * 10, 150.0);  // 相机位置 = 焦点 + 向量

        // 设置相机参数
        view->setCameraPosition(movepos.x, movepos.y, movepos.z, pos.x, pos.y, pos.z, 0, 0, 1);
        view->updateCamera();

        std::stringstream ss;
        ss << pic_dir.toLocal8Bit().data() << "\\局部碰撞" << i + 1 << ".png";
        view->saveScreenshot(ss.str());
        view->removeShape(balls_[i].id);
        view->removeShape(balls_[i].id + "sphere");
        view->removeText3D("ball" + balls_[i].id + "#");


        // 计算相机位置
        view->addSphere(balls_[i].cen, balls_[i].radiu, 1, 1, 0, balls_[i].id);
        view->addSphere(balls_[i].nearst.linept, 3, 1, 0, 0, balls_[i].id + "sphere");
        view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, balls_[i].id);
        view->addText3D(showtext, balls_[i].cen, 8.0, 1, 0, 0, "ball" + balls_[i].id + "#");
        vec camerapos = obb.pos + seevec * cameralen*0.5;  // 相机位置 = 焦点 + 向量
        view->setCameraPosition(camerapos.x, camerapos.y, camerapos.z, obb.pos.x, obb.pos.y, obb.pos.z, 0, 0, 1);
        view->updateCamera();
        // 截图
        ss.clear();
        ss.str("");
        ss << pic_dir.toLocal8Bit().data() << "\\局部碰撞" << i + 1 << "总图.png";
        view->saveScreenshot(ss.str());

        view->removeShape(balls_[i].id);
        view->removeText3D("ball" + balls_[i].id + "#");
        view->removeShape(balls_[i].id + "sphere");
        std::cout << showtext << balls_[i].cen << "ball" + balls_[i].id + "#" << std::endl;
    }

    // 添加左下角点云信息
    std::stringstream minstr, maxstr, counts;
    //counts << "counts: " << src_cloud_->size();
    minstr << std::fixed << setprecision(3) << "UTMmin: " << cloudmin.x << " " << cloudmin.y << " " << cloudmin.z;
    maxstr << std::fixed << setprecision(3) << "UTMmax: " << cloudmax.x << " " << cloudmax.y << " " << cloudmax.z;
    //view->addText(counts.str(), 5, 30, 14, 1, 0, 1, std::string("counts"));
    view->addText(minstr.str(), 5, 24, 14, 0, 0, 1, std::string("aabbBoxmin"));
    view->addText(maxstr.str(), 5, 10, 14, 0, 0, 1, std::string("aabbBoxmax"));

    // 添加左上角碰撞信息
    QString fmt = QStringLiteral("crash position:\nno   lon    lat     h   CrashRadius\n");
    for (int i = 0; i < balls_.size(); ++i)
    {
        fmt += QString::number(i + 1) + QStringLiteral(" ") + QString::fromStdString(balls_[i].description);
    }
    int lineHeight = 16;
    double vp[4];
    vtkRenderer *render = view->getRenderWindow()->GetRenderers()->GetFirstRenderer();
    render->GetViewport(vp);
    render->NormalizedDisplayToDisplay(vp[0], vp[1]);
    render->NormalizedDisplayToDisplay(vp[2], vp[3]);
    double dy = vp[3] - vp[1];
    if (!view->addText(fmt.toUtf8().data(), lineHeight, (std::max)(0, (int)dy - (4 + (int)balls_.size()) * lineHeight), lineHeight, 0, 0, 1, "crashText"))
        view->updateText(fmt.toUtf8().data(), lineHeight, (std::max)(0, (int)dy - (4 + (int)balls_.size()) * lineHeight), lineHeight, 0, 0, 1, "crashText");

    for (int i = 0; i < balls_.size(); ++i)
    {
        std::string showtext = balls_[i].id;
        pct::StringReplace(showtext, "ball", "");
        //view->addSphere(balls_[i].cen, balls_[i].radiu, 1, 1, 0, balls_[i].id);
        //view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, balls_[i].id);
        //view->addText3D(showtext, balls_[i].cen, 8.0, 1, 0, 0, "ball" + balls_[i].id + "#"); 
        view->addText3D(showtext, balls_[i].cen, 8.0, 1, 0, 0, "ball" + balls_[i].id + "#");
        view->addSphere(balls_[i].nearst.linept, 3, 1, 0, 0, balls_[i].id);
    }


    // 计算相机位置
    view->setCameraPosition(camerapos.x, camerapos.y, camerapos.z, obb.pos.x, obb.pos.y, obb.pos.z, 0, 0, 1);
    view->updateCamera();
    // 截图
    view->saveScreenshot(std::string(pic_dir.toLocal8Bit().data()) + "\\总图.png");
    std::cout << "截图完成" << std::endl;

    //// 调试观看
    //while (!view->wasStopped())
    //{
    //    view->spinOnce(100);
    //    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //}


    // 导出json
    std::map<std::string, std::tuple<float, float>> class_diastances = setting.get_distances();
    std::stringstream sstr;
    sstr.setf(ios::fixed, ios::floatfield);
    sstr << fixed << setprecision(1);
    sstr << std::get<0>(class_diastances["ground"]) << "," << std::get<1>(class_diastances["ground"]);
    boost::property_tree::ptree pt;
    pt.put("距离要求.地面", sstr.str());
    sstr.clear();
    sstr.str("");
    sstr << std::get<0>(class_diastances["veget"]) << "," << std::get<1>(class_diastances["veget"]);
    pt.put("距离要求.植被", sstr.str());
    pt.put("图例颜色.地面", setting.cls_strcolor(ground_str));
    pt.put("图例颜色.铁塔", setting.cls_strcolor(tower_str));
    pt.put("图例颜色.电力线", setting.cls_strcolor(power_line_str));
    pt.put("图例颜色.植被", setting.cls_strcolor(veget_str));
    pt.put("xy平面图路径", "images/总图.png");
    boost::property_tree::ptree errpt_array;

    std::cout << "遍历balls" << balls_.size() << std::endl;
     for (int i = 0; i < balls_.size(); ++i)
     {
         boost::property_tree::ptree errpt_child;
         sstr.clear();
         sstr.str("");
         streamCat<int>(sstr, i+1);
         errpt_child.put("序号", sstr.str());
         streamCat<std::string>(sstr, balls_[i].lineno);
         errpt_child.put("塔杆区间", sstr.str());
         sstr.clear();
         sstr.str("");
         double x = balls_[i].cen.x;
         double y = balls_[i].cen.y;
         pct::UTMXY2LatLon(x, y);
         sstr << std::fixed << setprecision(8) << x << "," << y << "," << std::fixed << setprecision(1) << balls_[i].cen.z;
         std::cout << sstr.str() << std::endl;
         errpt_child.put("隐患坐标", sstr.str());

         sstr << std::fixed << setprecision(1);
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
         streamCat<std::string>(sstr, std::string("images") + "/局部碰撞" + QString::number(i + 1).toLocal8Bit().data() + ".png");
         errpt_child.put("细节图", sstr.str());
         streamCat<std::string>(sstr, std::string("images") + "/局部碰撞" + QString::number(i + 1).toLocal8Bit().data() + "总图.png");
         errpt_child.put("总图", sstr.str());
         errpt_array.push_back(std::make_pair("", errpt_child));
     }
     pt.put_child("隐患列表", errpt_array);
     std::string json_path = setting.outputdir + "\\" + pct::ExtractExeName(setting.inputfile) + "检测结果.json";
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

double DangerousDistanceCheck::CollisionBall::GetExtraBoxRadiu(const pcl::PointXYZRGB &pt)
{
    pcl::PointXYZRGB max_t = max;
    pcl::PointXYZRGB min_t = min;


    if (pt.x > max_t.x)
        max_t.x = pt.x;
    if (pt.y > max_t.y)
        max_t.y = pt.y;
    if (pt.z > min_t.z)
        min_t.z = pt.z;

    return sqrt(pow(min_t.x - max_t.x, 2) + pow(min_t.y - max_t.y, 2) + pow(min_t.z - max_t.z, 2)) / 2 + 1;
}
