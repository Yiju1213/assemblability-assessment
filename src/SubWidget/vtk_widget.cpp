// PCLVisulizer -> QVTKOpenGLNativeWidget
#include <pcl/visualization/interactor_style.h>
#include <qvtkopenglnativewidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QSurfaceFormat>

// Qt Module
#include <QDebug>
#include <QMessageBox>


// VTK Module
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include "ui_vtk_widget.h"
#include "vtk_widget.h"

VTKWidget::VTKWidget(QWidget* parent)
    : parent_(parent),
      ui_(new Ui::VTKWidget) {
  //
  using pcl::visualization::PCLVisualizerInteractorStyle;
  //
  ui_->setupUi(this);
  vtkObject::GlobalWarningDisplayOff();
  // PCLVisulizer 嵌入到 QVTKOpenGLNativeWidget
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(0.3, 0.3, 0.3);

  auto renderWindowInteractor = vtkSmartPointer<QVTKInteractor>::New();
  auto interator_style = PCLVisualizerInteractorStyle::New();
  renderWindowInteractor->SetInteractorStyle(interator_style);

  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetInteractor(renderWindowInteractor);

  ui_->vtk_widget->setRenderWindow(renderWindow);

  // 挂载方便查询
  viewer_renderer_ = renderer;
  viewer_renderwindow_ = renderWindow;

  pcl_viewer_.reset(new pcl::visualization::PCLVisualizer(
      ui_->vtk_widget->GetRenderWindow()->GetRenderers()->GetFirstRenderer(),
      ui_->vtk_widget->GetRenderWindow(),
      "viewer",
      false));
  pcl_viewer_->setupInteractor(ui_->vtk_widget->GetRenderWindow()->GetInteractor(),
                               ui_->vtk_widget->GetRenderWindow());
  pcl_viewer_->addCoordinateSystem(0.5, "cloud", 0);
}

VTKWidget::~VTKWidget() {
  delete ui_;
}

void VTKWidget::renderCloudProperty(QVariant cloud_obj, CloudProperty property, Visibility visible) {
  qDebug() << __func__;

  if (cloud_obj.canConvert<XyzPointCloudPropertySharedPtr>()) {
    qDebug() << "Render Mesh Cloud";
    auto cloud_prop = cloud_obj.value<XyzPointCloudPropertySharedPtr>();
    auto cloud_id = cloud_prop->getCloudId();

    if (property == CloudProperty::CloudPoints) {
      // 三维点
      std::string points_viewer_name = std::to_string(cloud_id) + "Points";

      if (visible) {
        auto& cloud_points = cloud_prop->getCloudPoints();
        auto& cloud_color = cloud_prop->getCloudColor();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            source_cloud_color_handler(cloud_points, cloud_color.red(), cloud_color.green(), cloud_color.blue());
        pcl_viewer_->addPointCloud(cloud_points, source_cloud_color_handler, points_viewer_name);
      } else {
        pcl_viewer_->removePointCloud(points_viewer_name);
      }

    } else if (property == CloudProperty::CloudNormals) {
      // 法线
      std::string normals_viewer_name = std::to_string(cloud_id) + "Normals";

      if (visible) {
        auto& cloud_points = cloud_prop->getCloudPoints();
        auto& cloud_normals = cloud_prop->getCloudNormals();
        pcl_viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
            cloud_points, cloud_normals, 20, 0.02, normals_viewer_name);
      } else {
        pcl_viewer_->removePointCloud(normals_viewer_name);
      }

    } else if (property == CloudProperty::CloudFPFHs) {
      // FPFH

    } else if (property == CloudProperty::CloudColor) {
      // 用update而不是add
      auto& cloud_points = cloud_prop->getCloudPoints();
      auto& cloud_color = cloud_prop->getCloudColor();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          new_cloud_color_handler(cloud_points, cloud_color.red(), cloud_color.green(), cloud_color.blue());
      pcl_viewer_->updatePointCloud(cloud_points, new_cloud_color_handler, std::to_string(cloud_id) + "Points");
    }

  }

  else if (cloud_obj.canConvert<XyzMeshCloudPropertySharedPtr>()) {
    qDebug() << "Render Mesh Cloud";
    // Mesh
    auto cloud_prop = cloud_obj.value<XyzMeshCloudPropertySharedPtr>();

    if (property == CloudProperty::CloudMesh) {
      if (visible) {
        viewer_renderer_->AddActor(cloud_prop->getMeshActor());
      } else {
        viewer_renderer_->RemoveActor(cloud_prop->getMeshActor());
      }
    }
  
  } else {
    // ...
  }

  viewer_renderwindow_->Render();
}

void VTKWidget::renderCloudMesh() {
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
  reader->SetFileName("test.vtk");

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  viewer_renderer_->AddActor(actor);

  viewer_renderwindow_->Render();
}