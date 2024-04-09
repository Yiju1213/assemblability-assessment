#pragma once

#include <vtkActor.h>
#include <vtkSmartPointer.h>

#include <QColor>
#include <QMetaType>
#include <pclwrapper/PCLWrapper.hpp>

#include "GlobalConfig/global_config.h"



class vtkActor;
template <typename Tp>
class vtkSmartPointer;

template <typename PointT>
class PointCloudProperty {
  using CloudPointsSharedPtr = std::shared_ptr<pcl::PointCloud<PointT>>;
  using CloudNormalsSharedPtr = std::shared_ptr<pcl::PointCloud<pcl::Normal>>;
  using CloudFPFHSharedPtr = std::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>>;

 public:
  PointCloudProperty(const size_t id)
      : id_(id) {}
  // : cloud_name_(""),
  //   point_type_(""),
  //   cloud_color_(),
  //   cloud_points_shared_ptr_(nullptr),
  //   cloud_normals_shared_ptr_(nullptr) {}

  PointCloudProperty(const size_t id,
                     std::string cloud_name,
                     std::string point_type,
                     QColor cloud_color,
                     CloudPointsSharedPtr cloud_points_shared_ptr)
      : id_(id),
        cloud_name_(cloud_name),
        point_type_(point_type),
        cloud_color_(cloud_color),
        cloud_points_shared_ptr_(cloud_points_shared_ptr),
        cloud_normals_shared_ptr_(nullptr),
        cloud_fpfh_shared_ptr_(nullptr) {}

  const size_t& getCloudId() {
    return id_;
  }

  const std::string& getCloudName() {
    return cloud_name_;
  }

  const CloudPointsSharedPtr& getCloudPoints() {
    return cloud_points_shared_ptr_;
  }

  const CloudNormalsSharedPtr& getCloudNormals() {
    return cloud_normals_shared_ptr_;
  }

  const CloudFPFHSharedPtr& getCloudFPFHs() {
    return cloud_fpfh_shared_ptr_;
  }

  const QColor& getCloudColor() {
    return cloud_color_;
  }

  void updateCloudColor(QColor new_cloud_color) {
    cloud_color_ = new_cloud_color;
  }

 private:
  friend class PclProcess;

  void updateCloudName(std::string new_cloud_name) {
    cloud_name_ = new_cloud_name;
  }

  void updateCloudPoints(CloudPointsSharedPtr new_cloud_points_shared_ptr) {
    cloud_points_shared_ptr_.reset();
    cloud_points_shared_ptr_ = new_cloud_points_shared_ptr;
  }

  void updateCloudNormals(CloudNormalsSharedPtr new_cloud_normals_shared_ptr) {
    cloud_normals_shared_ptr_.reset();
    cloud_normals_shared_ptr_ = new_cloud_normals_shared_ptr;
  }

  void updateCloudFPFHs(CloudFPFHSharedPtr new_cloud_fpfh_shared_ptr) {
    cloud_fpfh_shared_ptr_.reset();
    cloud_fpfh_shared_ptr_ = new_cloud_fpfh_shared_ptr;
  }

  // std::string file_path_;
  const size_t id_;
  size_t points_number;
  std::string cloud_name_;
  std::string point_type_;
  QColor cloud_color_;
  CloudPointsSharedPtr cloud_points_shared_ptr_;
  CloudNormalsSharedPtr cloud_normals_shared_ptr_;
  CloudFPFHSharedPtr cloud_fpfh_shared_ptr_;
};

using XyzPointCloudProperty = PointCloudProperty<pcl::PointXYZ>;
using XyzPointCloudPropertySharedPtr = std::shared_ptr<PointCloudProperty<pcl::PointXYZ>>;

class XyzCloudPairCorrspondences {
 public:
  XyzCloudPairCorrspondences(const size_t src_id, const size_t tar_id, pcl::CorrespondencesPtr corrs)
      : src_id_(src_id), tar_id_(tar_id), corrs_(corrs) {}

 private:
  // 只用id保证了每次要获取相应property的时候都会检查是否已经被删除
  const size_t src_id_;
  const size_t tar_id_;
  // XyzPointCloudPropertySharedPtr src_property;
  // XyzPointCloudPropertySharedPtr tar_property;
  pcl::CorrespondencesPtr corrs_;

  friend class XyzCloudDB;
};

using XyzCloudPairCorrsSharedPtr = std::shared_ptr<XyzCloudPairCorrspondences>;

class XyzMeshCloudProperty {
 public:
  XyzMeshCloudProperty(const size_t mesh_id, std::string mesh_name, vtkSmartPointer<vtkActor> mesh_actor)
      : mesh_id_(mesh_id), mesh_name_(mesh_name), mesh_actor_(mesh_actor) {}

  size_t getMeshId() {
    return mesh_id_;
  }

  const vtkSmartPointer<vtkActor>& getMeshActor() {
    return mesh_actor_;
  }

  const std::string& getMeshName() {
    return mesh_name_;
  }

  const QColor& getMeshColor() {
    return mesh_color_;
  }

  void updateMeshColor(QColor new_color) {
    mesh_color_ = new_color;
  }

 private:
  friend class VtkProcess;

  void updateMeshActor(vtkSmartPointer<vtkActor> new_mesh_actor) {
    mesh_actor_ = new_mesh_actor;
  }

  void updateMeshName(std::string new_name) {
    mesh_name_ = new_name;
  }

  const size_t mesh_id_;
  std::string mesh_name_;
  QColor mesh_color_;
  vtkSmartPointer<vtkActor> mesh_actor_;
};

using XyzMeshCloudPropertySharedPtr = std::shared_ptr<XyzMeshCloudProperty>;

// * 单例模式数据库
class XyzCloudDB {
 public:
  static XyzCloudDB* getInstance() {
    return &XyzCloudDB_;
  }

  // * XyzCloudProperty
  size_t registerXyzPointCloudProperty(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_shared_ptr,
                                       std::string cloud_name,
                                       QColor cloud_color);

  const XyzPointCloudPropertySharedPtr& queryXyzPointCloudProperty(size_t id);

  void deleteXyzPointCloudProperty(size_t id);

  // * XyzCloudPairCorrespondences
  void registerXyzCloudPairCorrspondences(const size_t src_id,
                                          const size_t tar_id,
                                          pcl::CorrespondencesPtr corrs);

  const XyzCloudPairCorrsSharedPtr& queryXyzCloudPairCorrspondences(const size_t src_id, const size_t tar_id);

  void deleteXyzCloudPairCorrespondences(const size_t delete_id);

  // * XyzCloudMeshProperty
  size_t registerXyzMeshCloudProperty(vtkSmartPointer<vtkActor> mesh_actor_shared_ptr,
                              std::string mesh_name,
                              QColor mesh_color);
                        
  const XyzMeshCloudPropertySharedPtr& queryXyzMeshCloudProperty(size_t id);

  void deleteXyzMeshCloudProperty(size_t id);

 private:
  size_t next_assign_id = 0;
  std::map<size_t, XyzPointCloudPropertySharedPtr> point_cloud_property_map;
  std::map<std::pair<size_t, size_t>, XyzCloudPairCorrsSharedPtr> cloud_pair_corrs_map;
  std::map<size_t, XyzMeshCloudPropertySharedPtr> mesh_cloud_property_map;

  //
  XyzCloudDB(){};
  XyzCloudDB(XyzCloudDB const&) = delete;
  XyzCloudDB& operator=(XyzCloudDB const&) = delete;
  static XyzCloudDB XyzCloudDB_;
};

Q_DECLARE_METATYPE(XyzPointCloudPropertySharedPtr)
Q_DECLARE_METATYPE(XyzMeshCloudPropertySharedPtr)
