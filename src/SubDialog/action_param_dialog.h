#pragma once

#include "GlobalConfig/global_config.h"

#include <QString>
#include <QVector3D>
class QDialog;

// 包装成函数形式 因为参数设置都是一次性存在的
namespace action_param_dialog {
  using global_config::PointUnit;
  // using global_config::TitleLevel1;
  // using global_config::TitleLevel2;
  // using global_config::Label;

  struct NormalEstimationParam {
    NormalEstimationParam()
        : search_method_(""),
          method_param_(0),
          viewpoint_(QVector3D()) {}

    NormalEstimationParam(
        QString search_method,
        double method_param,
        QVector3D viewpoint)
        : search_method_(search_method),
          method_param_(method_param),
          viewpoint_(viewpoint) {}

   public:
    bool isEmpty(void) {
      return (search_method_ == "");
    }

    const QString search_method_;
    const double method_param_;
    const QVector3D viewpoint_;
  };

  struct FeatureEstimationParam {
    FeatureEstimationParam()
        : feature_type_(""),
          search_method_(""),
          method_param_(0) {}

    FeatureEstimationParam(
        QString feature_type,
        QString search_method,
        double method_param)
        : feature_type_(feature_type),
          search_method_(search_method),
          method_param_(method_param) {}

   public:
    bool isEmpty(void) {
      return (search_method_ == "");
    }

    const QString feature_type_;
    const QString search_method_;
    const double method_param_;
  };

  struct RansacEstimationParam {
    RansacEstimationParam()
        : name_sort_list_(std::vector<QString>()),
          feature_base_(""),
          inlier_threshold_(0.0),
          max_iter_times_(0) {}

    RansacEstimationParam(
        std::vector<QString> name_sort_list,
        QString feature_base,
        double inlier_threshold,
        int max_iter_times)
        : name_sort_list_(name_sort_list),
          feature_base_(feature_base),
          inlier_threshold_(inlier_threshold),
          max_iter_times_(max_iter_times) {}

   public:
    bool isEmpty(void) {
      return (feature_base_ == "");
    }

    const std::vector<QString> name_sort_list_;
    const QString feature_base_;
    const double inlier_threshold_;
    const int max_iter_times_;
  };

  struct VoxelGridParam {
    VoxelGridParam()
        : leaf_cube_grid_size_(0.0),
          modify_on_source_cloud_(false) {}

    VoxelGridParam(double leaf_cube_grid_size, bool modify_on_source_cloud)
        : leaf_cube_grid_size_(leaf_cube_grid_size),
          modify_on_source_cloud_(modify_on_source_cloud) {}

   public:
    bool isEmpty(void) {
      return (leaf_cube_grid_size_ == 0.0);
    }

    const double leaf_cube_grid_size_;
    const bool modify_on_source_cloud_;
  };

  struct UnitConvertParam {
    UnitConvertParam()
        : from_(PointUnit::None), to_(PointUnit::None) {}

    UnitConvertParam(PointUnit from, PointUnit to)
        : from_(from), to_(to) {}

   public:
    bool isEmpty(void) {
      return (from_ == to_ == PointUnit::None);
    }

    PointUnit from_;
    PointUnit to_;
  };

  NormalEstimationParam dialogNormalEstimationParam(QDialog* dialog);

  FeatureEstimationParam dialogFeatureEstimationParam(QDialog* dialog);

  RansacEstimationParam dialogRansacEstimationParam(QDialog* dialog, const std::vector<QString>& name_list);

  VoxelGridParam dialogVoxelGridFilterParam(QDialog* dialog);
}  // namespace action_param_dialog
