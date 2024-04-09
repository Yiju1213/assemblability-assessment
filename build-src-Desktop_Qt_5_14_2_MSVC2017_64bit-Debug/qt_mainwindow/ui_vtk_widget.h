/********************************************************************************
** Form generated from reading UI file 'vtk_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VTK_WIDGET_H
#define UI_VTK_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <qvtkopenglnativewidget.h>

QT_BEGIN_NAMESPACE

class Ui_VTKWidget
{
public:
    QGridLayout *gridLayout_2;
    QVTKOpenGLNativeWidget *vtk_widget;

    void setupUi(QWidget *VTKWidget)
    {
        if (VTKWidget->objectName().isEmpty())
            VTKWidget->setObjectName(QString::fromUtf8("VTKWidget"));
        VTKWidget->resize(400, 300);
        gridLayout_2 = new QGridLayout(VTKWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        vtk_widget = new QVTKOpenGLNativeWidget(VTKWidget);
        vtk_widget->setObjectName(QString::fromUtf8("vtk_widget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(vtk_widget->sizePolicy().hasHeightForWidth());
        vtk_widget->setSizePolicy(sizePolicy);
        vtk_widget->setLayoutDirection(Qt::LeftToRight);
        vtk_widget->setAutoFillBackground(true);

        gridLayout_2->addWidget(vtk_widget, 0, 0, 1, 1);


        retranslateUi(VTKWidget);

        QMetaObject::connectSlotsByName(VTKWidget);
    } // setupUi

    void retranslateUi(QWidget *VTKWidget)
    {
        VTKWidget->setWindowTitle(QCoreApplication::translate("VTKWidget", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class VTKWidget: public Ui_VTKWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VTK_WIDGET_H
