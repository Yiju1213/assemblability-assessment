#pragma once
// 只适配 pcl::PointCloudXYZ

// 为了使用信号槽机制
#include <QWidget>

// Qt Template
#include <QString>
#include <QVector3D>

#include "GlobalConfig/global_config.h"
#include "PropertyDataBase/property_database.h"

class PclProcess : public QWidget {
  Q_OBJECT
  //
  using CloudProperty = global_config::CloudProperty;
  using Visibility = global_config::Visibility;
  using ItemOperation = global_config::ItemOperation;
  using TreeList = global_config::TreeList;
  using XyzCloudPoints = pcl::PointCloud<pcl::PointXYZ>;
  using XyzCloudPointsSharedPtr = std::shared_ptr<XyzCloudPoints>;
  //
 public:
  explicit PclProcess(QWidget* parent = nullptr);

  const XyzPointCloudPropertySharedPtr& getXyzPointCloudProperty(const size_t id);

  bool isPropertyExists(const size_t id, CloudProperty property);

  void loadPclFile(const std::string path,
                   const std::string path_suffix,
                   const std::string cloud_name,
                   QColor color_rgb);

  bool savePclFile(const size_t id,
                   const std::string abso_path,
                   const std::string suffix);

  void estimateXyzCloudNormals(const std::vector<size_t> interest_cloud_id_vector,
                               const QString search_method,
                               const double method_param,
                               const QVector3D viewpoint);

  bool estimateXyzCloudFPFHs(const std::vector<size_t> interest_cloud_id_vector,
                             const QString search_method,
                             const double method_param);

  bool estimateFeatureBasedRansacTransformation(const std::pair<size_t, size_t> cloud_id_pair,
                                                const QString feature_type,
                                                const double inlier_threshold,
                                                const int max_iter_times);

  void processVoxelGridFilter(const std::vector<size_t> interest_cloud_id_vector,
                              const double leaf_grid_cube_size,
                              const bool modify_on_source);

  bool mergeCloudPoints(const std::vector<size_t> interest_cloud_id_vector,
                        QColor output_color);

 signals:
  void callVisualizeProcess(size_t cloud_id, CloudProperty property, Visibility visible = Visibility::Show, TreeList cloud_type = TreeList::PointCloud);
  void callCloudItemEditOnScreen(size_t cloud_id, TreeList list, ItemOperation op, QString new_name = "");

 public slots:
  void editCloudItemOnDatabase(size_t cloud_id, ItemOperation op, QString new_name = "");

 private:
  std::shared_ptr<XyzCloudDB> cloud_database_;
};