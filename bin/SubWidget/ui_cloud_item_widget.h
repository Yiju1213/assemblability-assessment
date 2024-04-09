/********************************************************************************
** Form generated from reading UI file 'cloud_item_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CLOUD_ITEM_WIDGET_H
#define UI_CLOUD_ITEM_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CloudItemWidget
{
public:
    QAction *actionHideCloudPoints;
    QAction *actionShowCloudPoints;
    QAction *actionHideCloudNormals;
    QAction *actionShowCloudNormals;
    QAction *actionDeleteItem;
    QAction *actionRename;
    QHBoxLayout *horizontalLayout;
    QCheckBox *checkBox;
    QToolButton *moreButton;

    void setupUi(QWidget *CloudItemWidget)
    {
        if (CloudItemWidget->objectName().isEmpty())
            CloudItemWidget->setObjectName(QString::fromUtf8("CloudItemWidget"));
        CloudItemWidget->resize(346, 164);
        actionHideCloudPoints = new QAction(CloudItemWidget);
        actionHideCloudPoints->setObjectName(QString::fromUtf8("actionHideCloudPoints"));
        actionShowCloudPoints = new QAction(CloudItemWidget);
        actionShowCloudPoints->setObjectName(QString::fromUtf8("actionShowCloudPoints"));
        actionHideCloudNormals = new QAction(CloudItemWidget);
        actionHideCloudNormals->setObjectName(QString::fromUtf8("actionHideCloudNormals"));
        actionShowCloudNormals = new QAction(CloudItemWidget);
        actionShowCloudNormals->setObjectName(QString::fromUtf8("actionShowCloudNormals"));
        actionDeleteItem = new QAction(CloudItemWidget);
        actionDeleteItem->setObjectName(QString::fromUtf8("actionDeleteItem"));
        actionRename = new QAction(CloudItemWidget);
        actionRename->setObjectName(QString::fromUtf8("actionRename"));
        horizontalLayout = new QHBoxLayout(CloudItemWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(5, 0, 5, 0);
        checkBox = new QCheckBox(CloudItemWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        horizontalLayout->addWidget(checkBox);

        moreButton = new QToolButton(CloudItemWidget);
        moreButton->setObjectName(QString::fromUtf8("moreButton"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(moreButton->sizePolicy().hasHeightForWidth());
        moreButton->setSizePolicy(sizePolicy);
        moreButton->setPopupMode(QToolButton::InstantPopup);
        moreButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        moreButton->setAutoRaise(false);
        moreButton->setArrowType(Qt::NoArrow);

        horizontalLayout->addWidget(moreButton);


        retranslateUi(CloudItemWidget);

        QMetaObject::connectSlotsByName(CloudItemWidget);
    } // setupUi

    void retranslateUi(QWidget *CloudItemWidget)
    {
        CloudItemWidget->setWindowTitle(QCoreApplication::translate("CloudItemWidget", "Form", nullptr));
        actionHideCloudPoints->setText(QCoreApplication::translate("CloudItemWidget", "HideCloudPoints", nullptr));
        actionShowCloudPoints->setText(QCoreApplication::translate("CloudItemWidget", "ShowCloudPoints", nullptr));
        actionHideCloudNormals->setText(QCoreApplication::translate("CloudItemWidget", "HideCloudNormals", nullptr));
        actionShowCloudNormals->setText(QCoreApplication::translate("CloudItemWidget", "ShowCloudNormals", nullptr));
        actionDeleteItem->setText(QCoreApplication::translate("CloudItemWidget", "DeleteItem", nullptr));
        actionRename->setText(QCoreApplication::translate("CloudItemWidget", "Rename", nullptr));
        checkBox->setText(QString());
        moreButton->setText(QCoreApplication::translate("CloudItemWidget", "<more> ", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CloudItemWidget: public Ui_CloudItemWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CLOUD_ITEM_WIDGET_H
