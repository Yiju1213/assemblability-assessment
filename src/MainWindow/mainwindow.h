#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

// PCL Data Process
#include "PointCloudProcess/point_cloud_process.h"
// VTK Data Process
#include "MeshProcess/mesh_process.h"
// FCL Collide Detect Process
#include "CollideDetect/bvhMeshClosestPointsDetector.h"
// PCL Visualization
#include <pcl/visualization/pcl_visualizer.h>

#include "GlobalConfig/global_config.h"

QT_BEGIN_NAMESPACE
namespace Ui {
  class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT
  using PCLVisualizer = pcl::visualization::PCLVisualizer;
  using CloudProperty = global_config::CloudProperty;
  using Visibility = global_config::Visibility;
  using ItemOperation = global_config::ItemOperation;
  using TreeList = global_config::TreeList;

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 signals:
  void callCloudItemEditOnDatabase(size_t cloud_id, ItemOperation op, QString new_name = "");  // 数据库操作转接信号

 private slots:
  // seld-defined
  void visualizeCloudProperty(size_t cloud_id, CloudProperty property, Visibility visible, TreeList cloud_type);

  void editCloudItemOnScreen(size_t cloud_id, TreeList list, ItemOperation op, QString new_name = "");  // 界面操作

  void editCloudItem(size_t cloud_id, TreeList list, ItemOperation op, QString new_name = "");  // 界面操作+转接到数据库操作

  // auto-generate
  void on_actionOpenFiles_triggered();

  void on_actionSaveFiles_triggered();

  void on_actionNormalEstimation_triggered();

  void on_actionFPFH_Estimation_triggered();

  void on_actionRANSAC_RoughEstimation_triggered();

  void on_actionChangeColor_triggered();

  void on_actionVoxelGridFilter_triggered();

  void on_actionMergeClouds_triggered();

  void on_actionConvertUnit_triggered();

  void on_actionCollideDetection_triggered();

private:
  inline QString isItemsPropertyAllExist(std::vector<size_t> id_list, CloudProperty property);
  inline std::vector<size_t> findPressedCloudIds(TreeList on_list);
  inline int isCloudItemOnTree(TreeList on_list, size_t cloud_id);

  Ui::MainWindow *ui_;
  std::shared_ptr<PclProcess> pcl_processor_;
  std::shared_ptr<VtkProcess> vtk_processor_;
};

#endif  // MAINWINDOW_H
