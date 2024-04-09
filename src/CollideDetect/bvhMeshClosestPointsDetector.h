#ifndef BVHMESHCLOSESTPOINTSDETECTOR_H
#define BVHMESHCLOSESTPOINTSDETECTOR_H

#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyData;

namespace fcl {
  template <typename BV>
  class BVHModel;
  template <typename S_>
  class OBBRSS;
  template <typename S>
  class CollisionObject;
  // using CollisionObjectf = CollisionObject<float>;
}  // namespace fcl

namespace AssemblyEvaluationCore {
  typedef fcl::BVHModel<fcl::OBBRSS<float>> fclBvhModel;
  typedef fcl::CollisionObject<float> fclCollisionObject;

  struct DistanceResult {
   public:
   bool isCollision;
    double distance;
    double pointOnA[3];
    double pointOnB[3];
  };

  struct CollisionResult {
    public:
     
    

  } ;

  class bvhMeshClosestPointsDetector {
   public:
    bvhMeshClosestPointsDetector(vtkSmartPointer<vtkActor> vtkActorA, vtkSmartPointer<vtkActor> vtkActorB);
    const DistanceResult distance();

   private:
    bvhMeshClosestPointsDetector();
    std::shared_ptr<fclBvhModel> vtkPolyDataToFclBvhModel(vtkSmartPointer<vtkPolyData> polyData);
    void syncTransform(vtkSmartPointer<vtkActor> from, std::shared_ptr<fclCollisionObject> to);

   protected:
    vtkSmartPointer<vtkActor> m_vtkActorA;
    vtkSmartPointer<vtkActor> m_vtkActorB;

    std::shared_ptr<fclCollisionObject> m_collisionObjectA;
    std::shared_ptr<fclCollisionObject> m_collisionObjectB;
  };
}  // namespace AssemblyEvaluationCore

#endif  // BVHMESHCLOSESTPOINTSDETECTOR_H
