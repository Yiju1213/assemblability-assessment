#ifndef CLOUDITEMWIDGET_H
#define CLOUDITEMWIDGET_H

#include <QWidget>

#include "GlobalConfig/global_config.h"

namespace Ui {
  class CloudItemWidget;
}

class CloudItemWidget : public QWidget {
  Q_OBJECT
  using CloudProperty = global_config::CloudProperty;
  using Visibility = global_config::Visibility;
  using ItemOperation = global_config::ItemOperation;
  using TreeList = global_config::TreeList;

 public:
  explicit CloudItemWidget(const size_t cloud_id, TreeList list, QWidget *parent = nullptr, QString cloud_name = "");
  ~CloudItemWidget();

  bool isRadioButtonPressed(void);

  QString getItemName(void);

  size_t getItemId(void) {
    return cloud_id_;
  }

  void updateItemName(QString new_name);

 signals:
  void callVisualizeProcess(size_t cloud_id, CloudProperty property, Visibility visible, TreeList on_list);

  void callCloudItemEdit(size_t cloud_id, TreeList list, ItemOperation op, QString new_name = "");

 private slots:
  void on_toolButton_clicked();

  void on_actionHideCloudPoints_triggered();

  void on_actionShowCloudPoints_triggered();

  void on_actionHideCloudNormals_triggered();

  void on_actionShowCloudNormals_triggered();

  void on_actionDeleteItem_triggered();

  void on_actionRename_triggered();

private:
  Ui::CloudItemWidget *ui;
  const size_t cloud_id_; 
  TreeList on_list_;
};

#endif  // CLOUDITEMWIDGET_H
