#ifndef VTK_WIDGET_H
#define VTK_WIDGET_H

#include <QWidget>
#include <QVariant>

// PCL Visualization
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include "PropertyDataBase/property_database.h"
#include "GlobalConfig/global_config.h"

// class pcl::visualization::PCLVisualizer;
// template <typename PointInT>
// class PointCloudProperty;

namespace Ui {
  class VTKWidget;
}

class VTKWidget : public QWidget {
  Q_OBJECT
  using XyzCloudPropertySharedPtr = std::shared_ptr<PointCloudProperty<pcl::PointXYZ>>;
  using PCLVisualizer = pcl::visualization::PCLVisualizer;
  using CloudProperty = global_config::CloudProperty;
  using Visibility = global_config::Visibility;

 public:
  explicit VTKWidget(QWidget *parent = nullptr);
  ~VTKWidget();

 public slots:
  void renderCloudProperty(QVariant cloud_obj, CloudProperty property, Visibility visible);
  void renderCloudMesh();

 private:
  Ui::VTKWidget *ui_;
  QWidget *parent_;
  std::shared_ptr<PCLVisualizer> pcl_viewer_;
  vtkSmartPointer<vtkRenderer> viewer_renderer_;
  vtkSmartPointer<vtkRenderWindow> viewer_renderwindow_;
};

#endif  // VTK_WIDGET_H
