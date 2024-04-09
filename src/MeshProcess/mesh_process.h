#pragma once

// 为了使用信号槽机制
#include <QColor>
#include <QWidget>

#include "PropertyDataBase/property_database.h"
#include "GlobalConfig/global_config.h"

class VtkProcess : public QWidget {
  Q_OBJECT
  using ItemOperation = global_config::ItemOperation;
  using CloudProperty = global_config::CloudProperty;
  using Visibility = global_config::Visibility;
  using TreeList = global_config::TreeList;

 public:
  explicit VtkProcess(QWidget* parent = nullptr);

  const XyzMeshCloudPropertySharedPtr& getXyzMeshCloudProperty(const size_t id);

  bool isPropertyExists(const size_t id, CloudProperty property);

  void loadVtkFile(const std::string mesh_path, const std::string mesh_name, const QColor mesh_color);

  void drawLine(double point_A[3], double point_B[3]);

 signals:
  void callVisualizeProcess(size_t cloud_id, CloudProperty property, Visibility visible = Visibility::Show, TreeList cloud_type = TreeList::MeshCloud);
  void callCloudItemEditOnScreen(size_t cloud_id, TreeList list, ItemOperation op, QString new_name = "");

  public slots:
  void editCloudItemOnDatabase(size_t cloud_id, ItemOperation op, QString new_name = "");

 private:
  std::shared_ptr<XyzCloudDB> cloud_database_;
};