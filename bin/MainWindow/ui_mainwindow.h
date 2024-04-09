/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableView>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidget>
#include "SubWidget/vtk_widget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpenFiles;
    QAction *actionSaveFiles;
    QAction *actionVoxelGridFilter;
    QAction *actionPlaneSegment;
    QAction *actionPassThrough_Filter;
    QAction *actionStatisticalOutlierRemoval_filter;
    QAction *actionRANSAC_RoughEstimation;
    QAction *actionICP_Accurate_Estimation;
    QAction *actionMatrix_Transformation;
    QAction *actionNormalEstimation;
    QAction *actionFPFH_Estimation;
    QAction *actionChangeColor;
    QAction *actionMergeClouds;
    QAction *actionCollideDetection;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QTreeWidget *cloud_item_tree;
    VTKWidget *vtk_widget;
    QTableView *cloud_property_table;
    QLabel *label_2;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QMenuBar *menubar;
    QMenu *menuFiles;
    QMenu *menuPreProcess;
    QMenu *menuAlign;
    QMenu *menuEdit;
    QMenu *menuNormal_Feature;
    QMenu *menuMesh;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1041, 634);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        actionOpenFiles = new QAction(MainWindow);
        actionOpenFiles->setObjectName(QString::fromUtf8("actionOpenFiles"));
        actionSaveFiles = new QAction(MainWindow);
        actionSaveFiles->setObjectName(QString::fromUtf8("actionSaveFiles"));
        actionVoxelGridFilter = new QAction(MainWindow);
        actionVoxelGridFilter->setObjectName(QString::fromUtf8("actionVoxelGridFilter"));
        actionPlaneSegment = new QAction(MainWindow);
        actionPlaneSegment->setObjectName(QString::fromUtf8("actionPlaneSegment"));
        actionPassThrough_Filter = new QAction(MainWindow);
        actionPassThrough_Filter->setObjectName(QString::fromUtf8("actionPassThrough_Filter"));
        actionStatisticalOutlierRemoval_filter = new QAction(MainWindow);
        actionStatisticalOutlierRemoval_filter->setObjectName(QString::fromUtf8("actionStatisticalOutlierRemoval_filter"));
        actionRANSAC_RoughEstimation = new QAction(MainWindow);
        actionRANSAC_RoughEstimation->setObjectName(QString::fromUtf8("actionRANSAC_RoughEstimation"));
        actionICP_Accurate_Estimation = new QAction(MainWindow);
        actionICP_Accurate_Estimation->setObjectName(QString::fromUtf8("actionICP_Accurate_Estimation"));
        actionMatrix_Transformation = new QAction(MainWindow);
        actionMatrix_Transformation->setObjectName(QString::fromUtf8("actionMatrix_Transformation"));
        actionNormalEstimation = new QAction(MainWindow);
        actionNormalEstimation->setObjectName(QString::fromUtf8("actionNormalEstimation"));
        actionFPFH_Estimation = new QAction(MainWindow);
        actionFPFH_Estimation->setObjectName(QString::fromUtf8("actionFPFH_Estimation"));
        actionChangeColor = new QAction(MainWindow);
        actionChangeColor->setObjectName(QString::fromUtf8("actionChangeColor"));
        actionMergeClouds = new QAction(MainWindow);
        actionMergeClouds->setObjectName(QString::fromUtf8("actionMergeClouds"));
        actionCollideDetection = new QAction(MainWindow);
        actionCollideDetection->setObjectName(QString::fromUtf8("actionCollideDetection"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        cloud_item_tree = new QTreeWidget(centralwidget);
        QFont font;
        font.setFamily(QString::fromUtf8("Times New Roman"));
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setTextAlignment(0, Qt::AlignCenter);
        __qtreewidgetitem->setFont(0, font);
        cloud_item_tree->setHeaderItem(__qtreewidgetitem);
        QBrush brush(QColor(170, 255, 255, 255));
        brush.setStyle(Qt::NoBrush);
        QFont font1;
        font1.setPointSize(10);
        QTreeWidgetItem *__qtreewidgetitem1 = new QTreeWidgetItem(cloud_item_tree);
        __qtreewidgetitem1->setFont(0, font1);
        __qtreewidgetitem1->setBackground(0, brush);
        QTreeWidgetItem *__qtreewidgetitem2 = new QTreeWidgetItem(cloud_item_tree);
        __qtreewidgetitem2->setFont(0, font1);
        cloud_item_tree->setObjectName(QString::fromUtf8("cloud_item_tree"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(cloud_item_tree->sizePolicy().hasHeightForWidth());
        cloud_item_tree->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(cloud_item_tree, 1, 0, 1, 1);

        vtk_widget = new VTKWidget(centralwidget);
        vtk_widget->setObjectName(QString::fromUtf8("vtk_widget"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(2);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(vtk_widget->sizePolicy().hasHeightForWidth());
        vtk_widget->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(vtk_widget, 0, 1, 4, 1);

        cloud_property_table = new QTableView(centralwidget);
        cloud_property_table->setObjectName(QString::fromUtf8("cloud_property_table"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(1);
        sizePolicy3.setVerticalStretch(1);
        sizePolicy3.setHeightForWidth(cloud_property_table->sizePolicy().hasHeightForWidth());
        cloud_property_table->setSizePolicy(sizePolicy3);
        cloud_property_table->setFont(font1);

        gridLayout->addWidget(cloud_property_table, 3, 0, 1, 1);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(1);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy4);
        QFont font2;
        font2.setFamily(QString::fromUtf8("Times New Roman"));
        font2.setPointSize(10);
        font2.setBold(true);
        font2.setWeight(75);
        label_2->setFont(font2);
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1041, 31));
        QFont font3;
        font3.setPointSize(12);
        font3.setBold(false);
        font3.setUnderline(false);
        font3.setWeight(50);
        font3.setKerning(true);
        menubar->setFont(font3);
        menuFiles = new QMenu(menubar);
        menuFiles->setObjectName(QString::fromUtf8("menuFiles"));
        menuPreProcess = new QMenu(menubar);
        menuPreProcess->setObjectName(QString::fromUtf8("menuPreProcess"));
        menuAlign = new QMenu(menubar);
        menuAlign->setObjectName(QString::fromUtf8("menuAlign"));
        menuEdit = new QMenu(menubar);
        menuEdit->setObjectName(QString::fromUtf8("menuEdit"));
        menuNormal_Feature = new QMenu(menubar);
        menuNormal_Feature->setObjectName(QString::fromUtf8("menuNormal_Feature"));
        menuMesh = new QMenu(menubar);
        menuMesh->setObjectName(QString::fromUtf8("menuMesh"));
        MainWindow->setMenuBar(menubar);

        menubar->addAction(menuFiles->menuAction());
        menubar->addAction(menuEdit->menuAction());
        menubar->addAction(menuPreProcess->menuAction());
        menubar->addAction(menuNormal_Feature->menuAction());
        menubar->addAction(menuAlign->menuAction());
        menubar->addAction(menuMesh->menuAction());
        menuFiles->addAction(actionOpenFiles);
        menuFiles->addAction(actionSaveFiles);
        menuPreProcess->addAction(actionVoxelGridFilter);
        menuPreProcess->addAction(actionPassThrough_Filter);
        menuPreProcess->addAction(actionStatisticalOutlierRemoval_filter);
        menuPreProcess->addSeparator();
        menuPreProcess->addAction(actionPlaneSegment);
        menuAlign->addAction(actionRANSAC_RoughEstimation);
        menuAlign->addAction(actionICP_Accurate_Estimation);
        menuEdit->addAction(actionMatrix_Transformation);
        menuEdit->addAction(actionChangeColor);
        menuEdit->addAction(actionMergeClouds);
        menuNormal_Feature->addAction(actionNormalEstimation);
        menuNormal_Feature->addAction(actionFPFH_Estimation);
        menuMesh->addAction(actionCollideDetection);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpenFiles->setText(QCoreApplication::translate("MainWindow", "Open Files...", nullptr));
#if QT_CONFIG(statustip)
        actionOpenFiles->setStatusTip(QCoreApplication::translate("MainWindow", "Open Point Cloud File...", nullptr));
#endif // QT_CONFIG(statustip)
        actionSaveFiles->setText(QCoreApplication::translate("MainWindow", "Save Files...", nullptr));
        actionVoxelGridFilter->setText(QCoreApplication::translate("MainWindow", "Voxel Grid Filter", nullptr));
#if QT_CONFIG(tooltip)
        actionVoxelGridFilter->setToolTip(QCoreApplication::translate("MainWindow", "Voxel Grid Filter", nullptr));
#endif // QT_CONFIG(tooltip)
        actionPlaneSegment->setText(QCoreApplication::translate("MainWindow", "Plane Segment", nullptr));
        actionPassThrough_Filter->setText(QCoreApplication::translate("MainWindow", "PassThrough Filter", nullptr));
        actionStatisticalOutlierRemoval_filter->setText(QCoreApplication::translate("MainWindow", "StatisticalOutlierRemoval filter\n"
"\n"
"", nullptr));
        actionRANSAC_RoughEstimation->setText(QCoreApplication::translate("MainWindow", "RANSAC Rough Estimation", nullptr));
        actionICP_Accurate_Estimation->setText(QCoreApplication::translate("MainWindow", "ICP Accurate Estimation", nullptr));
        actionMatrix_Transformation->setText(QCoreApplication::translate("MainWindow", "Matrix Transformation", nullptr));
        actionNormalEstimation->setText(QCoreApplication::translate("MainWindow", "Normal Estimation", nullptr));
        actionFPFH_Estimation->setText(QCoreApplication::translate("MainWindow", "FPFH Estimation", nullptr));
        actionChangeColor->setText(QCoreApplication::translate("MainWindow", "Change Color", nullptr));
        actionMergeClouds->setText(QCoreApplication::translate("MainWindow", "Merge Clouds", nullptr));
        actionCollideDetection->setText(QCoreApplication::translate("MainWindow", "Collide Detection", nullptr));
        QTreeWidgetItem *___qtreewidgetitem = cloud_item_tree->headerItem();
        ___qtreewidgetitem->setText(0, QCoreApplication::translate("MainWindow", "Cloud Item List", nullptr));

        const bool __sortingEnabled = cloud_item_tree->isSortingEnabled();
        cloud_item_tree->setSortingEnabled(false);
        QTreeWidgetItem *___qtreewidgetitem1 = cloud_item_tree->topLevelItem(0);
        ___qtreewidgetitem1->setText(0, QCoreApplication::translate("MainWindow", "Point Cloud", nullptr));
        QTreeWidgetItem *___qtreewidgetitem2 = cloud_item_tree->topLevelItem(1);
        ___qtreewidgetitem2->setText(0, QCoreApplication::translate("MainWindow", "Mesh Cloud", nullptr));
        cloud_item_tree->setSortingEnabled(__sortingEnabled);

#if QT_CONFIG(statustip)
        cloud_property_table->setStatusTip(QCoreApplication::translate("MainWindow", "Interacting with cloud property table", nullptr));
#endif // QT_CONFIG(statustip)
        label_2->setText(QCoreApplication::translate("MainWindow", "Cloud Property Table", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        menuFiles->setTitle(QCoreApplication::translate("MainWindow", "Files", nullptr));
        menuPreProcess->setTitle(QCoreApplication::translate("MainWindow", "PreProcess", nullptr));
        menuAlign->setTitle(QCoreApplication::translate("MainWindow", "Align", nullptr));
        menuEdit->setTitle(QCoreApplication::translate("MainWindow", "Edit", nullptr));
        menuNormal_Feature->setTitle(QCoreApplication::translate("MainWindow", "Characterize", nullptr));
        menuMesh->setTitle(QCoreApplication::translate("MainWindow", "Collide", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
