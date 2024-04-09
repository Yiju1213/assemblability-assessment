#include "action_param_dialog.h"


// Qt Widget
#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QRadioButton>
#include <QString>
#include <QVector3D>
// Qt Layout
#include <QFormLayout>
#include <QHBoxLayout>

namespace action_param_dialog {

  // 外部传入栈空间Dialog对象 外部函数出栈时Dialog挂载的子Widget全部销毁
  // 如果内部堆空间创造Dialog对象
  NormalEstimationParam dialogNormalEstimationParam(QDialog* dialog) {
    // 多输入对话框获取法线估计参数设置信息
    QFormLayout form(dialog);
    QLabel* title = new QLabel("Normal Estimation Parameters Input:");
    title->setFont(global_config::TitleLevel1);
    form.addRow(title);

    // Value: 搜索方式 + 方式参数
    QHBoxLayout* layout_search_method = new QHBoxLayout();
    QLabel* label_search_method = new QLabel("Search Method:", dialog);
    QLabel* label_method_param = new QLabel("Method Param:", dialog);
    QComboBox* combobox_search_method = new QComboBox(dialog);
    QDoubleSpinBox* spinbox_method_param = new QDoubleSpinBox(dialog);
    QStringList list_search_method;

    label_search_method->setFont(global_config::TitleLevel2);
    label_method_param->setFont(global_config::TitleLevel2);
    label_search_method->setAlignment(Qt::AlignmentFlag::AlignLeft);
    label_method_param->setAlignment(Qt::AlignmentFlag::AlignRight);
    list_search_method << "<None>"
                       << global_config::SearchMethodList;
    combobox_search_method->setFont(global_config::Label);                   
    combobox_search_method->addItems(list_search_method);
    spinbox_method_param->setFont(global_config::Label);
    spinbox_method_param->setDecimals(3);  // 3位小数
    spinbox_method_param->setValue(0.00);
    // spinbox_method_param->setSuffix("");

    layout_search_method->addWidget(label_search_method);
    layout_search_method->addWidget(combobox_search_method);
    layout_search_method->addWidget(label_method_param);
    layout_search_method->addWidget(spinbox_method_param);

    form.addRow(layout_search_method);

    // Value: 视角
    QHBoxLayout* layout_viewpoint = new QHBoxLayout();
    QLabel* label_viewpoint = new QLabel("Viewpoint at(mm):", dialog);
    QStringList dimension;
    QVector<QLineEdit*> dimension_lines(3);

    label_viewpoint->setFont(global_config::TitleLevel2);

    layout_viewpoint->addWidget(label_viewpoint);

    dimension << "x"
              << "y"
              << "z";
    for (size_t i = 0; i < dimension.size(); ++i) {
      QLineEdit* line = new QLineEdit(dialog);
      line->setFont(global_config::Label);
      line->setPlaceholderText(("coordinate " + dimension[i]));
      line->setValidator(new QDoubleValidator(dialog));
      dimension_lines[i] = line;
      layout_viewpoint->addWidget(line);
    }

    form.addRow(layout_viewpoint);

    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, dialog);
    form.addRow(&buttonBox);

    QObject::connect(&buttonBox, &QDialogButtonBox::accepted, dialog, [=]() {
      if (combobox_search_method->currentText() == "<None>" ||
          spinbox_method_param->value() == 0.0) {
        QMessageBox::warning(
            dialog,
            "Error",
            "Parameter Setting Error!");
      } else {
        dialog->accept();
      }
    });
    QObject::connect(&buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

    // Process when OK button is clicked
    if (dialog->exec() == QDialog::Accepted) {
      // Get Retrivals
      return NormalEstimationParam(
          combobox_search_method->currentText(),
          spinbox_method_param->value(),
          QVector3D(dimension_lines[0]->text().toFloat(),
                    dimension_lines[1]->text().toFloat(),
                    dimension_lines[2]->text().toFloat()));

    } else {
      // 不进行下一步
      QMessageBox::warning(
          dialog,
          "Error",
          "Normal Estimation Terminate!");

      return NormalEstimationParam();
    }
  }

  FeatureEstimationParam dialogFeatureEstimationParam(QDialog* dialog) {
    // 多输入对话框获取特征点匹配参数设置信息
    QFormLayout form(dialog);
    QLabel* title = new QLabel("Feature Estimation Parameters Input:");
    title->setFont(global_config::TitleLevel1);
    form.addRow(title);

    QHBoxLayout* layout_feature_type = new QHBoxLayout();

    // Value: 搜索方式 + 方式参数
    QLabel* label_search_method = new QLabel("Search Method:", dialog);
    QComboBox* combobox_search_method = new QComboBox(dialog);
    QStringList list_search_method;

    label_search_method->setFont(global_config::TitleLevel2);
    label_search_method->setAlignment(Qt::AlignmentFlag::AlignLeft);
    list_search_method << "<None>" << global_config::SearchMethodList;
    combobox_search_method->setFont(global_config::Label);
    combobox_search_method->addItems(list_search_method);

    layout_feature_type->addWidget(label_search_method);
    layout_feature_type->addWidget(combobox_search_method);

    QLabel* label_method_param = new QLabel("Method Param:", dialog);
    QDoubleSpinBox* spinbox_method_param = new QDoubleSpinBox(dialog);
    label_method_param->setFont(global_config::TitleLevel2);
    label_method_param->setAlignment(Qt::AlignmentFlag::AlignRight);
    spinbox_method_param->setFont(global_config::Label);
    spinbox_method_param->setDecimals(3);  // 3位小数
    spinbox_method_param->setValue(0.00);

    layout_feature_type->addWidget(label_method_param);
    layout_feature_type->addWidget(spinbox_method_param);

    form.addRow(layout_feature_type);

    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, dialog);
    form.addRow(&buttonBox);

    QObject::connect(&buttonBox, &QDialogButtonBox::accepted, dialog, [=]() {
      if (combobox_search_method->currentText() == "<None>" ||
          spinbox_method_param->value() == 0.0) {
        QMessageBox::warning(
            dialog,
            "Error",
            "Parameter Setting Error!");
      } else {
        dialog->accept();
      }
    });
    QObject::connect(&buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

    // Process when OK button is clicked
    if (dialog->exec() == QDialog::Accepted) {
      // Get Retrivals
      return FeatureEstimationParam(
          "FPFH",
          combobox_search_method->currentText(),
          spinbox_method_param->value());

    } else {
      // 不进行下一步
      QMessageBox::warning(
          dialog,
          "Error",
          "Correspondences Estimation Terminate!");

      return FeatureEstimationParam();
    }
  }

  RansacEstimationParam dialogRansacEstimationParam(QDialog* dialog,
                                                    const std::vector<QString>& name_list) {
    // 多输入对话框获取特征点匹配参数设置信息
    QFormLayout form(dialog);

    // 标题
    QLabel* title = new QLabel("RANSAC Rough Estimation Parameters Input:");
    title->setFont(global_config::TitleLevel1);
    form.addRow(title);

    // 排序框
    QListWidget* sort_widget = new QListWidget(dialog);

    sort_widget->setDragDropMode(QAbstractItemView::InternalMove);

    for (auto& each_name : name_list) {
      QListWidgetItem* each_item = new QListWidgetItem(each_name, sort_widget);
      each_item->setTextAlignment(Qt::AlignCenter);
      sort_widget->addItem(each_item);
      // more...
    }

    form.addRow(sort_widget);

    // 特征依据
    QHBoxLayout* layout_feature_base = new QHBoxLayout();
    QLabel* label_feature_base = new QLabel("Based on feature:", dialog);
    QComboBox* combobox_feature_base = new QComboBox(dialog);
    QStringList list_feature_base;
    list_feature_base << "<None>" << global_config::LocalFeatureType;

    label_feature_base->setFont(global_config::TitleLevel2);
    combobox_feature_base->setFont(global_config::Label);
    combobox_feature_base->addItems(list_feature_base);

    layout_feature_base->addWidget(label_feature_base);
    layout_feature_base->addWidget(combobox_feature_base);

    form.addRow(layout_feature_base);

    // 内点阈值
    QHBoxLayout* layout_inlier_threshold = new QHBoxLayout();
    QLabel* label_inlier_threshold = new QLabel("Inlier threshold:", dialog);
    QLineEdit* line_inlier_threshold = new QLineEdit(dialog);

    label_inlier_threshold->setFont(global_config::TitleLevel2);
    line_inlier_threshold->setFont(global_config::Label);
    line_inlier_threshold->setPlaceholderText("单位:m");
    line_inlier_threshold->setValidator(new QDoubleValidator(dialog));

    layout_inlier_threshold->addWidget(label_inlier_threshold);
    layout_inlier_threshold->addWidget(line_inlier_threshold);

    form.addRow(layout_inlier_threshold);

    // 最大迭代次数
    QHBoxLayout* layout_max_iter_times = new QHBoxLayout();
    QLabel* label_max_iter_times = new QLabel("Max iteration times:", dialog);
    QLineEdit* line_max_iter_times = new QLineEdit(dialog);

    label_max_iter_times->setFont(global_config::TitleLevel2);
    line_max_iter_times->setFont(global_config::Label);
    line_max_iter_times->setValidator(new QIntValidator(dialog));

    layout_max_iter_times->addWidget(label_max_iter_times);
    layout_max_iter_times->addWidget(line_max_iter_times);

    form.addRow(layout_max_iter_times);

    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, dialog);
    form.addRow(&buttonBox);

    QObject::connect(&buttonBox, &QDialogButtonBox::accepted, dialog, [=]() {
      if (combobox_feature_base->currentText() == "<None>" ||
          line_inlier_threshold->text().toFloat() == 0.0 ||
          line_max_iter_times->text().toInt() <= 0) {
        QMessageBox::warning(
            dialog,
            "Error",
            "Parameter Setting Error!");
      } else {
        dialog->accept();
      }
    });
    QObject::connect(&buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

    // Process when OK button is clicked
    if (dialog->exec() == QDialog::Accepted) {
      // Get Retrivals
      std::vector<QString> sort_list;
      auto item_num = sort_widget->count();

      for (size_t i = 0; i < item_num; ++i)
        sort_list.push_back(sort_widget->item(i)->text());

      if (sort_list.size() != 2) {
        // error
        QMessageBox::warning(
            dialog,
            "Error",
            "RANSAC Estimation Terminate!");
        return RansacEstimationParam();
      }

      return RansacEstimationParam(
          sort_list,
          combobox_feature_base->currentText(),
          line_inlier_threshold->text().toFloat(),
          line_max_iter_times->text().toInt());
    } else {
      // 不进行下一步
      QMessageBox::warning(
          dialog,
          "Error",
          "RANSAC Estimation Terminate!");

      return RansacEstimationParam();
    }
  }

  VoxelGridParam dialogVoxelGridFilterParam(QDialog* dialog) {
    // 多输入对话框获取特征点匹配参数设置信息
    QFormLayout form(dialog);

    // 标题
    QLabel* title = new QLabel("Voxel Filter Parameters Input:");
    title->setFont(global_config::TitleLevel1);
    form.addRow(title);

    // Value: 体素半径
    QHBoxLayout* layout_leaf_size = new QHBoxLayout();
    QLabel* label_leaf_size = new QLabel("Leaf Cube Size(m):", dialog);
    QLineEdit* line_leaf_size = new QLineEdit(dialog);

    label_leaf_size->setFont(global_config::TitleLevel2);
    line_leaf_size->setFont(global_config::Label);
    line_leaf_size->setValidator(new QDoubleValidator(dialog));

    layout_leaf_size->addWidget(label_leaf_size);
    layout_leaf_size->addWidget(line_leaf_size);

    form.addRow(layout_leaf_size);

    // Value: 是否在源点云上进行更改
    QHBoxLayout* layout_on_src = new QHBoxLayout();
    QLabel* label_on_src = new QLabel("Modify on source cloud(True if pressd):", dialog);
    QRadioButton* button_on_src = new QRadioButton(dialog);

    label_on_src->setFont(global_config::TitleLevel2);
    button_on_src->setFont(global_config::Label);
    layout_on_src->addWidget(label_on_src);
    layout_on_src->addWidget(button_on_src);

    form.addRow(layout_on_src);

    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                               Qt::Horizontal, dialog);
    form.addRow(&buttonBox);

    QObject::connect(&buttonBox, &QDialogButtonBox::accepted, dialog, [=]() {
      if (line_leaf_size->text() == "") {
        QMessageBox::warning(
            dialog,
            "Error",
            "Parameter Setting Error!");
      } else {
        dialog->accept();
      }
    });
    QObject::connect(&buttonBox, &QDialogButtonBox::rejected, dialog, &QDialog::reject);

    // Process when OK button is clicked
    if (dialog->exec() == QDialog::Accepted) {
      // Get Retrivals
      return VoxelGridParam(line_leaf_size->text().toDouble(), button_on_src->isChecked());
    } else {
      // 不进行下一步
      QMessageBox::warning(
          dialog,
          "Error",
          "Voxel Filter Terminate!");

      return VoxelGridParam();
    }
  }

}  // namespace action_param_dialog
