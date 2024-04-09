#include "mainwindow.h"

#include "ui_mainwindow.h"
#include "SubWidget/cloud_item_widget.h"
#include "SubDialog/action_param_dialog.h"


// Qt Others
#include <QDebug>
#include <QFileInfo>
// Qt Dialog
#include <QColorDialog>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
// Qt Template
#include <QVariant>
#include <QVector>
// Qt Layout

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow),
      pcl_processor_(new PclProcess),
      vtk_processor_(new VtkProcess) {
  //
  using pcl::visualization::PCLVisualizerInteractorStyle;
  //
  ui_->setupUi(this);

  // PCL Process Signal/Slots
  connect(pcl_processor_.get(), &PclProcess::callVisualizeProcess, this, &MainWindow::visualizeCloudProperty);
  connect(pcl_processor_.get(), &PclProcess::callCloudItemEditOnScreen, this, &MainWindow::editCloudItemOnScreen);
  connect(this, &MainWindow::callCloudItemEditOnDatabase, pcl_processor_.get(), &PclProcess::editCloudItemOnDatabase);

  // VTK Process Signal/Slots
  connect(vtk_processor_.get(), &VtkProcess::callVisualizeProcess, this, &MainWindow::visualizeCloudProperty);
  connect(vtk_processor_.get(), &VtkProcess::callCloudItemEditOnScreen, this, &MainWindow::editCloudItemOnScreen);
  connect(this, &MainWindow::callCloudItemEditOnDatabase, vtk_processor_.get(), &VtkProcess::editCloudItemOnDatabase);
}

MainWindow::~MainWindow() {
  delete ui_;
}

inline std::vector<size_t> MainWindow::findPressedCloudIds(TreeList on_list) {
  // 遍历找到复选的项目
  std::vector<size_t> interest_items_id_vector;

  auto top_node = ui_->cloud_item_tree->topLevelItem(static_cast<int>(on_list));

  for (size_t child_node_index = 0; child_node_index < top_node->childCount(); ++child_node_index) {
    auto child_node = top_node->child(child_node_index);
    auto item_widget = static_cast<CloudItemWidget*>(ui_->cloud_item_tree->itemWidget(child_node, 0));

    if (item_widget->isRadioButtonPressed()) {
      qDebug() << "item " << child_node_index << " is pressed";
      qDebug() << "name " << item_widget->getItemName();
      //
      interest_items_id_vector.push_back(item_widget->getItemId());
    }
  }

  return interest_items_id_vector;
}

inline QString MainWindow::isItemsPropertyAllExist(std::vector<size_t> id_list, CloudProperty property) {
  QString problem_name_list;

  for (auto& each_id : id_list) {
    if (!pcl_processor_->isPropertyExists(each_id, property)) {
      problem_name_list += QString::fromStdString(pcl_processor_->getXyzPointCloudProperty(each_id)->getCloudName());
      problem_name_list += " ";
    }
  }

  return problem_name_list;
}

inline int MainWindow::isCloudItemOnTree(TreeList on_list, size_t cloud_id) {
  qDebug() << __func__;
  //
  auto top_node = ui_->cloud_item_tree->topLevelItem(static_cast<int>(on_list));
  qDebug() << on_list << "child count" << top_node->childCount();

  for (size_t child_node_index = 0; child_node_index < top_node->childCount(); ++child_node_index) {
    auto child_node = top_node->child(child_node_index);
    auto item_widget = static_cast<CloudItemWidget*>(ui_->cloud_item_tree->itemWidget(child_node, 0));

    if (item_widget->getItemId() == cloud_id) {
      qDebug() << cloud_id << " on " << top_node << " " << child_node;
      return child_node_index;
    }
  }

  return -1;
}

inline void MainWindow::editCloudItem(size_t cloud_id, TreeList list, ItemOperation op, QString new_name) {
  editCloudItemOnScreen(cloud_id, list, op, new_name);

  if (list == TreeList::PointCloud) {
    pcl_processor_->editCloudItemOnDatabase(cloud_id, op, new_name);
  } else if (list == TreeList::MeshCloud) {
    vtk_processor_->editCloudItemOnDatabase(cloud_id, op, new_name);
  } else {
    // ...
  }
}

void MainWindow::editCloudItemOnScreen(size_t cloud_id, TreeList list, ItemOperation op, QString new_name) {
  qDebug() << __func__;
  //
  if (op == ItemOperation::Add) {
    auto tree_item = new QTreeWidgetItem(ui_->cloud_item_tree->topLevelItem(static_cast<int>(list)));
    auto cloud_item_widget = new CloudItemWidget(cloud_id, list, this, new_name);

    ui_->cloud_item_tree->setItemWidget(tree_item, 0, cloud_item_widget);
    connect(cloud_item_widget, &CloudItemWidget::callVisualizeProcess, this, &MainWindow::visualizeCloudProperty);
    connect(cloud_item_widget, &CloudItemWidget::callCloudItemEdit, this, &MainWindow::editCloudItem);

  } else if (op == ItemOperation::Remove) {
    auto child_index = isCloudItemOnTree(list, cloud_id);
    if (child_index >= 0) {
      auto top_node = ui_->cloud_item_tree->topLevelItem(static_cast<int>(list));
      auto child_node = top_node->child(child_index);
      auto child_widget = ui_->cloud_item_tree->itemWidget(child_node, 0);

      ui_->cloud_item_tree->removeItemWidget(child_node, 0);
      top_node->takeChild(child_index);
      child_widget->deleteLater();
      delete child_node;

      visualizeCloudProperty(cloud_id, CloudProperty::CloudPoints, Visibility::Hide, list);
      visualizeCloudProperty(cloud_id, CloudProperty::CloudNormals, Visibility::Hide, list);
      visualizeCloudProperty(cloud_id, CloudProperty::CloudMesh, Visibility::Hide, list);
    }
  } else if (op == ItemOperation::Rename) {
    //
    auto child_index = isCloudItemOnTree(list, cloud_id);
    if (child_index >= 0) {
      auto top_node = ui_->cloud_item_tree->topLevelItem(static_cast<int>(list));
      auto child_widget = static_cast<CloudItemWidget*>(ui_->cloud_item_tree->itemWidget(top_node->child(child_index), 0));
      child_widget->updateItemName(new_name);
      qDebug() << "Rename on item tree";
    } else {
      qDebug() << "Rename fails";
    }
  }
}

void MainWindow::visualizeCloudProperty(size_t cloud_id, CloudProperty property, Visibility visible, TreeList cloud_type) {
  qDebug() << __func__;

  QVariant cloud_obj;

  if (cloud_type == TreeList::PointCloud) {
    if (pcl_processor_->isPropertyExists(cloud_id, property) == false) {
      qDebug() << "point property not exists";
      return;
    }

    XyzPointCloudPropertySharedPtr obj = pcl_processor_->getXyzPointCloudProperty(cloud_id);
    cloud_obj = QVariant::fromValue(obj);

  } else if (cloud_type == TreeList::MeshCloud) {
    if (vtk_processor_->isPropertyExists(cloud_id, property) == false) {
      qDebug() << "mesh property not exists";
      return;
    }

    XyzMeshCloudPropertySharedPtr obj = vtk_processor_->getXyzMeshCloudProperty(cloud_id);
    cloud_obj = QVariant::fromValue(obj);

  } else {
    return;
  }

  if (!cloud_obj.isNull())
    ui_->vtk_widget->renderCloudProperty(cloud_obj, property, visible);
  else
    qDebug() << "cloud_obj is Null";
}

void MainWindow::on_actionOpenFiles_triggered() {
  // 选择文件
  QString file_path = QFileDialog::getOpenFileName(
      this,
      tr("Open a Single File"),
      ".",
      tr("PLY Files(*.ply);;PCD Files(*.pcd);;VTK Files(*.vtk)"));

  QFileInfo file_info = QFileInfo(file_path);
  QString file_suffix = file_info.suffix();

  if (!file_suffix.isEmpty()) {
    // 输入名字
    bool name_button_ret = false;
    QString cloud_name = QInputDialog::getText(
        this,
        tr("Naming"),
        tr("Naming this loaded point cloud"),
        QLineEdit::EchoMode::Normal,
        QString(),
        &name_button_ret);

    if (name_button_ret && !cloud_name.isEmpty()) {
      // 颜色选择
      QColor cloud_color = QColorDialog::getColor(
          Qt::GlobalColor::white,
          this,
          "Selecting Cloud's Color");

      if (cloud_color.isValid()) {
        // 载入文件(大文件时候要改成多线程信号)
        if (file_suffix == "vtk") {
          vtk_processor_->loadVtkFile(
              file_path.toStdString(),
              cloud_name.toStdString(),
              cloud_color);
        } else {
          pcl_processor_->loadPclFile(
              file_path.toStdString(),
              file_suffix.toStdString(),
              cloud_name.toStdString(),
              cloud_color);
        }

      } else {
        // ...
      }
    } else {
      // ...
    }
  } else {
    // ...
  }
}

void MainWindow::on_actionSaveFiles_triggered() {  // TODO: MeshCloud Save
  qDebug() << __func__;

  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  if (interest_items_id_vector.size() != 1) {
    QMessageBox::warning(
        this,
        "Save Error",
        "Please select one item(and only)!");
    return;
  }

  // 文件保存对话框
  QString ret = QFileDialog::getSaveFileName(
      this,
      "Save a Single File",
      ".",
      "PLY Files(*.ply);;PCD Files(*.pcd);;VTK Files(*.vtk)");

  if (!ret.isEmpty()) {
    QFileInfo file_info = QFileInfo(ret);

    pcl_processor_->savePclFile(
        interest_items_id_vector[0],
        QDir::toNativeSeparators(file_info.filePath()).toStdString(),
        file_info.suffix().toStdString());
  }
}

void MainWindow::on_actionNormalEstimation_triggered() {
  qDebug() << __func__;

  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  if (!interest_items_id_vector.empty()) {
    QDialog param_dialog;
    auto param = action_param_dialog::dialogNormalEstimationParam(&param_dialog);

    if (!param.isEmpty()) {
      qDebug() << "normal estimation";
      // 新开线程计算法线
      pcl_processor_->estimateXyzCloudNormals(
          interest_items_id_vector,
          param.search_method_,
          param.method_param_,
          param.viewpoint_);
    } else {
      // already warning
    }
  } else {
    // nothing need to estimate
  }
}

void MainWindow::on_actionFPFH_Estimation_triggered() {
  qDebug() << __func__;
  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);
  std::string str = "need" + 1;
  std::cout << str << std::endl;
  if (!interest_items_id_vector.empty()) {
    // 确认都计算法线了，否则全部不计算直接退出
    auto problem_list = isItemsPropertyAllExist(interest_items_id_vector, CloudProperty::CloudNormals);

    if (!problem_list.isEmpty()) {
      QMessageBox::warning(
          this,
          "Missing Prependant Action",
          QString("Item \" %1\" need estimate normals first").arg(problem_list));
      return;
    }

    // 获得设置参数
    QDialog param_dialog;
    auto param = action_param_dialog::dialogFeatureEstimationParam(&param_dialog);

    if (!param.isEmpty()) {
      // 计算FPFH特征
      qDebug() << "feature type" << param.feature_type_;
      qDebug() << "search method" << param.search_method_;
      qDebug() << "method param" << param.method_param_;
      if (!pcl_processor_->estimateXyzCloudFPFHs(
              interest_items_id_vector,
              param.search_method_,
              param.method_param_)) {
        QMessageBox::warning(
            this,
            "Error",
            "FPFH Estimation Fails!");
      }
    } else {
      // ...
    }
  } else {
    // ...
  }
}

void MainWindow::on_actionRANSAC_RoughEstimation_triggered() {
  qDebug() << __func__;
  using action_param_dialog::dialogRansacEstimationParam;
  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  // 复选项目得多于2，且最终只能选择2个
  if (interest_items_id_vector.size() == 2) {
    // 获得设置参数
    QDialog param_dialog;
    std::vector<QString> name_list;

    for (auto& each_id : interest_items_id_vector)
      name_list.push_back(pcl_processor_->getXyzPointCloudProperty(each_id)->getCloudName().c_str());
    auto param = dialogRansacEstimationParam(&param_dialog, name_list);

    if (!param.isEmpty()) {
      // 判断参数中的特征依据是否存在
      if (param.feature_base_ == global_config::LocalFeatureType[0]) {  // FPFH
        auto problem_list = isItemsPropertyAllExist(interest_items_id_vector, CloudProperty::CloudFPFHs);

        if (!problem_list.isEmpty()) {
          QMessageBox::warning(
              this,
              "Missing Prependant Action",
              QString("Item \" %1\" need estimate FPFHs first").arg(problem_list));
          return;
        }
      } else if (param.feature_base_ == global_config::LocalFeatureType[1]) {  // SHOT
        // ...
        return;
      }
      // 进行RANSAC位姿粗估计
      if (param.name_sort_list_[0] != name_list[0]) {
        size_t tmp = interest_items_id_vector[1];
        interest_items_id_vector[1] = interest_items_id_vector[0];
        interest_items_id_vector[0] = tmp;
      }
      if (!pcl_processor_->estimateFeatureBasedRansacTransformation(
              std::make_pair(interest_items_id_vector[0], interest_items_id_vector[1]),
              param.feature_base_,
              param.inlier_threshold_,
              param.max_iter_times_)) {
        //
        QMessageBox::warning(
            this,
            "Error",
            "RANSAC Transformation Estimation Fails!");
      }
    } else {
      // ...
    }

  } else {
    QMessageBox::warning(
        this,
        "Error",
        "Please select only 2 items");
  }
}

void MainWindow::on_actionChangeColor_triggered() {
  qDebug() << __func__;
  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  for (auto& each_id : interest_items_id_vector) {
    QColor cloud_color = QColorDialog::getColor(
        Qt::GlobalColor::white,
        this,
        "Change Cloud's Color");

    if (cloud_color.isValid()) {
      // 通过数据库写入 随后在可视化窗口更新
      auto& cloud_property = pcl_processor_->getXyzPointCloudProperty(each_id);

      if (!cloud_property) {  // nullptr
        QMessageBox::warning(
            this,
            "Error",
            "nullptr item");
        return;
      }

      cloud_property->updateCloudColor(cloud_color);
      visualizeCloudProperty(each_id, CloudProperty::CloudColor, Visibility::Show, TreeList::PointCloud);
    } else {
      // ...
    }
  }
}

void MainWindow::on_actionVoxelGridFilter_triggered() {
  //
  qDebug() << __func__;
  using action_param_dialog::dialogVoxelGridFilterParam;
  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  if (!interest_items_id_vector.empty()) {
    // 获得设置参数
    QDialog param_dialog;
    auto param = dialogVoxelGridFilterParam(&param_dialog);

    if (!param.isEmpty()) {
      pcl_processor_->processVoxelGridFilter(
          interest_items_id_vector,
          param.leaf_cube_grid_size_,
          param.modify_on_source_cloud_);
    } else {
      // ...
    }
  } else {
    // ...
  }
}

void MainWindow::on_actionMergeClouds_triggered() {
  //
  qDebug() << __func__;
  // 遍历找到复选的项目
  auto interest_items_id_vector = findPressedCloudIds(TreeList::PointCloud);

  QColor cloud_color = QColorDialog::getColor(
      Qt::GlobalColor::white,
      this,
      "Change Cloud's Color");

  if (cloud_color.isValid()) {
    if (!pcl_processor_->mergeCloudPoints(interest_items_id_vector, cloud_color)) {
      QMessageBox::warning(
          this,
          "Error",
          "Merge somehow fails!");
    }
  } else {
    // ...
  }
}

void MainWindow::on_actionConvertUnit_triggered() {
  //
  qDebug() << __func__;
  //
}

void MainWindow::on_actionCollideDetection_triggered() {
  //
  qDebug() << __func__;
  //
  auto interest_items_id_vector = findPressedCloudIds(TreeList::MeshCloud);

  if (interest_items_id_vector.size() != 2) {
    QMessageBox::warning(
        this,
        "Error",
        "Mesh cloud item should be 2(and only)");
  } else {
    // 选定哪个是移动件 哪个是固定件
    auto actor_move = vtk_processor_->getXyzMeshCloudProperty(interest_items_id_vector[0])->getMeshActor();
    auto actor_fix = vtk_processor_->getXyzMeshCloudProperty(interest_items_id_vector[1])->getMeshActor();
    auto fcl_distance_detector = AssemblyEvaluationCore::bvhMeshClosestPointsDetector(actor_move, actor_fix);
    auto ret = fcl_distance_detector.distance();

    qDebug() << "min distance: " << ret.distance;
    qDebug() << "is Collide" << ret.isCollision;

    vtk_processor_->drawLine(ret.pointOnA, ret.pointOnB);
  }
}
