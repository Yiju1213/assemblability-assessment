#include "property_database.h"

#include <QDebug>

XyzCloudDB XyzCloudDB::XyzCloudDB_;

XyzPointCloudPropertySharedPtr null_point_property_shared_ptr;
XyzMeshCloudPropertySharedPtr null_mesh_property_shareed_ptr;
XyzCloudPairCorrsSharedPtr null_corrs_shared_ptr;


size_t XyzCloudDB::registerXyzPointCloudProperty(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_shared_ptr,
                                                 std::string cloud_name,
                                                 QColor cloud_color) {
  qDebug() << __func__;

  XyzPointCloudPropertySharedPtr
      this_cloud_property(new XyzPointCloudProperty(next_assign_id, cloud_name, typeid(pcl::PointXYZ).name(), cloud_color, cloud_shared_ptr));

  point_cloud_property_map.insert(std::make_pair(next_assign_id++, this_cloud_property));

  // test
  qDebug() << "newly register id" << next_assign_id - 1;

  return (next_assign_id - 1);
}

const XyzPointCloudPropertySharedPtr& XyzCloudDB::queryXyzPointCloudProperty(size_t id) {
  //
  auto& iter = point_cloud_property_map.find(id);
  if (iter != point_cloud_property_map.end()) {
    return iter->second;
  } else {
    return null_point_property_shared_ptr;
  }
}

void XyzCloudDB::deleteXyzPointCloudProperty(size_t id) {
  qDebug() << __func__;
  //
  auto property_node = queryXyzPointCloudProperty(id);

  point_cloud_property_map.erase(id);
}

void XyzCloudDB::registerXyzCloudPairCorrspondences(const size_t src_id,
                                                    const size_t tar_id,
                                                    pcl::CorrespondencesPtr corrs) {
  //
  qDebug() << __func__;
  // 先检查id是否有效
  if (!queryXyzPointCloudProperty(src_id) || !queryXyzPointCloudProperty(tar_id))
    return;

  XyzCloudPairCorrsSharedPtr this_corrs_pair(new XyzCloudPairCorrspondences(src_id, tar_id, corrs));

  cloud_pair_corrs_map[std::make_pair(src_id, tar_id)] = this_corrs_pair;  // 数据插入可以更新
}

const XyzCloudPairCorrsSharedPtr& XyzCloudDB::queryXyzCloudPairCorrspondences(const size_t src_id,
                                                                              const size_t tar_id) {
  //
  qDebug() << __func__;
  //
  auto& iter = cloud_pair_corrs_map.find(std::make_pair(src_id, tar_id));

  if (iter != cloud_pair_corrs_map.end()) {
    if (!queryXyzPointCloudProperty(src_id) || !queryXyzPointCloudProperty(tar_id))  // id对应点云被删除的情况
      return null_corrs_shared_ptr;
    return iter->second;
  } else {
    return null_corrs_shared_ptr;
  }
}

void XyzCloudDB::deleteXyzCloudPairCorrespondences(const size_t delete_id) {
  //
  qDebug() << __func__;
  //
  // O(2n)遍历
  for (size_t index = 0; index < next_assign_id; ++index) {
    auto& iter_on_src = cloud_pair_corrs_map.find(std::make_pair(delete_id, index));
    auto& iter_on_tar = cloud_pair_corrs_map.find(std::make_pair(index, delete_id));

    if (iter_on_src != cloud_pair_corrs_map.end()) {
      qDebug() << "find pair" << delete_id << " " << index;
      cloud_pair_corrs_map.erase(iter_on_src);
    }

    if (iter_on_tar != cloud_pair_corrs_map.end()) {
      qDebug() << iter_on_src->second.get();
      qDebug() << "find pair" << index << " " << delete_id;
      cloud_pair_corrs_map.erase(iter_on_tar);
    }
  }
}

size_t XyzCloudDB::registerXyzMeshCloudProperty(vtkSmartPointer<vtkActor> mesh_actor_shared_ptr,
                                        std::string mesh_name,
                                        QColor mesh_color) {
  //
  qDebug() << __func__;
  //
  XyzMeshCloudPropertySharedPtr
      this_cloud_mesh(new XyzMeshCloudProperty(next_assign_id, mesh_name, mesh_actor_shared_ptr));

  mesh_cloud_property_map.insert(std::make_pair(next_assign_id++, this_cloud_mesh));

  // test
  qDebug() << "newly register id" << next_assign_id - 1;

  return (next_assign_id - 1);
}

const XyzMeshCloudPropertySharedPtr& XyzCloudDB::queryXyzMeshCloudProperty(size_t id) {
  qDebug( )<< __func__;
  //
  auto& iter = mesh_cloud_property_map.find(id);
  if (iter != mesh_cloud_property_map.end()) {
    return iter->second;
  } else {
    return null_mesh_property_shareed_ptr;
  }
}

void XyzCloudDB::deleteXyzMeshCloudProperty(size_t id) {
  qDebug() << __func__;
  //
  auto property_node = queryXyzMeshCloudProperty(id);

  mesh_cloud_property_map.erase(id);
}