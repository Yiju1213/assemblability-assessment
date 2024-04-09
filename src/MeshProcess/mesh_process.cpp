#include "mesh_process.h"
//
#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkProperty.h>
//
#include <vtkLineSource.h>
//
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
//
#include <vtkSmartPointer.h>

// #include <vtkVector.h>
#include <QDebug>

VtkProcess::VtkProcess(QWidget* parent)
    : QWidget(parent), cloud_database_(XyzCloudDB::getInstance()) {}

const XyzMeshCloudPropertySharedPtr& VtkProcess::getXyzMeshCloudProperty(const size_t id) {
  return cloud_database_->queryXyzMeshCloudProperty(id);
}

bool VtkProcess::isPropertyExists(const size_t id, CloudProperty property) {
  auto cloud_property = getXyzMeshCloudProperty(id);

  if (!cloud_property)
    return false;

  switch (property) {
    case CloudProperty::CloudMesh:
      return (cloud_property->getMeshActor() != nullptr);
    case CloudProperty::CloudColor:
      return true;
    default:
      return false;
  }
}

void VtkProcess::loadVtkFile(const std::string mesh_path, const std::string mesh_name, const QColor mesh_color) {
  //
  qDebug() << __func__;
  // Mapper -> Actor <- Property
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
  reader->SetFileName(mesh_path.c_str());

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  actor->GetProperty()->SetColor(mesh_color.red()/255.0, mesh_color.green()/255.0, mesh_color.blue()/255.0);

  auto cloud_id = cloud_database_->registerXyzMeshCloudProperty(actor, mesh_name, mesh_color);

  emit callCloudItemEditOnScreen(cloud_id, TreeList::MeshCloud, ItemOperation::Add, mesh_name.c_str());
  emit callVisualizeProcess(cloud_id, CloudProperty::CloudMesh);
}

void VtkProcess::editCloudItemOnDatabase(size_t cloud_id, ItemOperation op, QString new_name) {
  qDebug() << __func__;

  if (op == ItemOperation::Remove) {
    cloud_database_->deleteXyzMeshCloudProperty(cloud_id);
  } else if (op == ItemOperation::Rename) {
    cloud_database_->queryXyzMeshCloudProperty(cloud_id)->updateMeshName(new_name.toStdString());
  } else {
    // ...
  }
}

void VtkProcess::drawLine(double point_A[3], double point_B[3]) {
  qDebug() << __func__;
  //
  auto line_source = vtkSmartPointer<vtkLineSource>::New();
  
  line_source->SetPoint1(point_A);
  line_source->SetPoint2(point_B);

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(line_source->GetOutputPort());
 
	auto line_actor = vtkSmartPointer<vtkActor>::New();

	line_actor->SetMapper(mapper);
	line_actor->GetProperty()->SetColor(1, 0, 0);
	line_actor->GetProperty()->SetLineWidth(3);

  auto cloud_id = cloud_database_->registerXyzMeshCloudProperty(line_actor, "line", QColor());
  auto new_name = "line_id" + std::to_string(cloud_id);

  getXyzMeshCloudProperty(cloud_id)->updateMeshName(new_name);

  emit callCloudItemEditOnScreen(cloud_id, TreeList::MeshCloud, ItemOperation::Add, new_name.c_str());
  emit callVisualizeProcess(cloud_id, CloudProperty::CloudMesh);
}