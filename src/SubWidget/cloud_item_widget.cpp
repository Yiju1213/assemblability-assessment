#include "cloud_item_widget.h"

#include "ui_cloud_item_widget.h"


#include <QDebug>
#include <QInputDialog>
#include <QMenu>
#include <QMessageBox>

CloudItemWidget::CloudItemWidget(const size_t cloud_id, TreeList list, QWidget* parent, QString cloud_name)
    : cloud_id_(cloud_id),
      on_list_(list),
      QWidget(parent),
      ui(new Ui::CloudItemWidget) {
  //
  ui->setupUi(this);

  //
  QMenu* menuSelection = new QMenu(this);
  menuSelection->addAction(ui->actionRename);
  menuSelection->addAction(ui->actionShowCloudPoints);
  menuSelection->addAction(ui->actionHideCloudPoints);
  menuSelection->addAction(ui->actionShowCloudNormals);
  menuSelection->addAction(ui->actionHideCloudNormals);
  menuSelection->addAction(ui->actionDeleteItem);
  ui->moreButton->setMenu(menuSelection);

  ui->checkBox->setText(cloud_name);
}

CloudItemWidget::~CloudItemWidget() {
  delete ui;
}

bool CloudItemWidget::isRadioButtonPressed(void) {
  return ui->checkBox->isChecked();
}

QString CloudItemWidget::getItemName(void) {
  return ui->checkBox->text();
}

void CloudItemWidget::updateItemName(QString new_name) {
  ui->checkBox->setText(new_name);
}

void CloudItemWidget::on_toolButton_clicked() {
  // TODO:动态支持可选项
}

void CloudItemWidget::on_actionHideCloudPoints_triggered() {
  emit callVisualizeProcess(cloud_id_, CloudProperty::CloudPoints, Visibility::Hide, on_list_);
}

void CloudItemWidget::on_actionShowCloudPoints_triggered() {
  emit callVisualizeProcess(cloud_id_, CloudProperty::CloudPoints, Visibility::Show, on_list_);
}

void CloudItemWidget::on_actionHideCloudNormals_triggered() {
  emit callVisualizeProcess(cloud_id_, CloudProperty::CloudNormals, Visibility::Hide, on_list_);
}

void CloudItemWidget::on_actionShowCloudNormals_triggered() {
  emit callVisualizeProcess(cloud_id_, CloudProperty::CloudNormals, Visibility::Show, on_list_);
}

void CloudItemWidget::on_actionDeleteItem_triggered() {
  // Warning
  auto ret_button = QMessageBox::question(
      this,
      "Confirming",
      "Sure to delete this item?",
      QMessageBox::Yes | QMessageBox::No,
      QMessageBox::No);

  if (ret_button == QMessageBox::Yes) {
    // auto& item_name = ui->radioButton->text();

    // 删除item信息
    emit callCloudItemEdit(cloud_id_, on_list_, ItemOperation::Remove);
    // 从item property窗口删除？
  } else {
    // ...
  }
}

void CloudItemWidget::on_actionRename_triggered() {
  qDebug() << __func__;
  // 重命名对话框
  auto new_name = QInputDialog::getText(this, "Rename Item", "Rename your item");
  if (new_name.isEmpty())
    return;
  // // 更改ItemWidget（为了一致性，不在这里处理）
  // updateItemName(new_name);
  // 更改其他窗口
  qDebug() << "input name" << new_name;
  emit callCloudItemEdit(cloud_id_, on_list_, ItemOperation::Rename, new_name);
}
