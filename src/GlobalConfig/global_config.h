#pragma once

class QStringList;
class QFont;

namespace global_config {
  extern QStringList SpatialStructList;
  extern QStringList SearchMethodList;
  extern QStringList LocalFeatureType;

  extern QFont TitleLevel1;
  extern QFont TitleLevel2;
  extern QFont Label;

  // enum PointCloudProperty {
  //   CloudPoints = 0,
  //   CloudNormals,
  //   CloudFPFHs,
  //   CloudColor,

  // };

  // enum MeshCloudProperty {
  //   CloudMesh = 0,
  //   CloudColor
  // };

  // union Property {
  //   using PointCloudP = PointCloudProperty;
  //   using MeshCloudP = MeshCloudProperty;
  //   PointCloudP point_cloud;
  //   MeshCloudP mesh_cloud;
  // };

  // extern Property CloudProperty;

  enum CloudProperty {
    CloudPoints = 0,
    CloudNormals,
    CloudFPFHs,
    CloudColor,
    CloudMesh
  };

  enum TreeList {
    PointCloud = 0,
    MeshCloud
  };

  enum Visibility {
    Hide = 0,
    Show
  };

  enum ItemOperation {
    Add = 0,
    Remove,
    Rename
  };

  enum PointUnit {
    None = 0,
    mm,
    cm,
    dm,
    m
  };

}  // namespace global_config
