#include "bvhMeshClosestPointsDetector.h"


#include <fcl/fcl.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCollisionDetectionFilter.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkLineSource.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>

#include <QDebug>

namespace AssemblyEvaluationCore {

  bvhMeshClosestPointsDetector::bvhMeshClosestPointsDetector(vtkSmartPointer<vtkActor> vtkActorA, vtkSmartPointer<vtkActor> vtkActorB)
      : m_vtkActorA(vtkActorA), m_vtkActorB(vtkActorB) {
    //
    qDebug() << __func__;
    //
    vtkSmartPointer<vtkPolyData> polyDataA = vtkPolyData::SafeDownCast(m_vtkActorA->GetMapper()->GetInputAsDataSet());
    vtkSmartPointer<vtkPolyData> polyDataB = vtkPolyData::SafeDownCast(m_vtkActorB->GetMapper()->GetInputAsDataSet());

    std::shared_ptr<fclBvhModel> bvhModelA = vtkPolyDataToFclBvhModel(polyDataA);
    std::shared_ptr<fclBvhModel> bvhModelB = vtkPolyDataToFclBvhModel(polyDataB);

    m_collisionObjectA = std::shared_ptr<fclCollisionObject>(new fclCollisionObject(bvhModelA));
    m_collisionObjectB = std::shared_ptr<fclCollisionObject>(new fclCollisionObject(bvhModelB));
  }

  bvhMeshClosestPointsDetector::bvhMeshClosestPointsDetector() {
  }

  std::shared_ptr<fclBvhModel> bvhMeshClosestPointsDetector::vtkPolyDataToFclBvhModel(vtkSmartPointer<vtkPolyData> polyData) {
    //
    qDebug() << __func__;
    //
    vtkCellArray *polys = polyData->GetPolys(); // 拓扑结构
    vtkPoints *points = polyData->GetPoints();  // 几何结构
    int totalVerts = points->GetNumberOfPoints();
    int totalTriangles = polys->GetNumberOfCells();

    qDebug() << "Total Vertices" << totalVerts;
    qDebug() << "Total Triangles" << totalTriangles;

    // set mesh triangles and vertice indices
    std::vector<fcl::Vector3f> vertices;
    std::vector<fcl::Triangle> triangles;

    // code to set the vertices and triangles

    // 添加顶点
    for (int i = 0; i < totalVerts; i++) {
      double *p = points->GetPoint(i);  // 对应的三个点的坐标值
      fcl::Vector3f fclPoint(p[0], p[1], p[2]);
      vertices.emplace_back(fclPoint);
    }

    // 添加三角面片
    for (int i = 0; i < totalTriangles; i++) {
      vtkSmartPointer<vtkIdList> pts = vtkSmartPointer<vtkIdList>::New();
      polys->GetCellAtId(i, pts);   // 三角面片对应的三个点的id
      fcl::Triangle tri(pts->GetId(0), pts->GetId(1), pts->GetId(2));
      triangles.emplace_back(tri);
    }
    // BVHModel is a template class for mesh geometry, for default OBBRSS template
    // is used

    std::shared_ptr<fclBvhModel> geom = std::make_shared<fclBvhModel>();
    // add the mesh data into the BVHModel structure
    geom->beginModel();
    geom->addSubModel(vertices, triangles);
    geom->endModel();

    return geom;
  }

  void bvhMeshClosestPointsDetector::syncTransform(vtkSmartPointer<vtkActor> from, std::shared_ptr<fclCollisionObject> to) {
    vtkMatrix4x4 *fromMatrix = from->GetMatrix();
    fcl::Matrix3f r;

    r << fromMatrix->GetElement(0, 0), fromMatrix->GetElement(0, 1), fromMatrix->GetElement(0, 2),
        fromMatrix->GetElement(1, 0), fromMatrix->GetElement(1, 1), fromMatrix->GetElement(1, 2),
        fromMatrix->GetElement(2, 0), fromMatrix->GetElement(2, 1), fromMatrix->GetElement(2, 2);

    fcl::Vector3f t;
    t << fromMatrix->GetElement(0, 3), fromMatrix->GetElement(1, 3), fromMatrix->GetElement(2, 3);

    to->setTransform(r, t);
  }

  const DistanceResult bvhMeshClosestPointsDetector::distance() {
    // 同步转换矩阵
    syncTransform(m_vtkActorA, m_collisionObjectA);
    syncTransform(m_vtkActorB, m_collisionObjectB);

    // 距离计算
    fcl::DistanceRequestf distance_request(true, true);
    // result will be returned via the collision result structure
    fcl::DistanceResultf distance_result;
    // perform distance test
    fcl::distance(m_collisionObjectA.get(), m_collisionObjectB.get(), distance_request, distance_result);

    // meshCollisionDetection();
    DistanceResult result;
    result.distance = distance_result.min_distance;

    result.pointOnA[0] = distance_result.nearest_points[0][0];
    result.pointOnA[1] = distance_result.nearest_points[0][1];
    result.pointOnA[2] = distance_result.nearest_points[0][2];

    result.pointOnB[0] = distance_result.nearest_points[1][0];
    result.pointOnB[1] = distance_result.nearest_points[1][1];
    result.pointOnB[2] = distance_result.nearest_points[1][2];

    fcl::CollisionRequestf collide_request(1Ui64, true);
    fcl::CollisionResultf collide_result;

    // perform collide test
    fcl::collide(m_collisionObjectA.get(), m_collisionObjectB.get(), collide_request, collide_result);

    result.isCollision = collide_result.isCollision();

    qDebug() << "contact " << collide_result.numContacts();
    qDebug() << "cost source " << collide_result.numCostSources();

    // CollisionResult collision_ret;
    // collision_ret.isCollision = collide_result.isCollision();
    // auto slide = collide_result.getContact(0);
    // fcl::Triangle *contact_tri =  (fcl::Triangle*)(slide.b1);

    return result;
  }
}  // namespace AssemblyEvaluationCore
