#include "point_cloud_process.h"

#include <QDebug>


// PCL Wrapper
#include <pclwrapper/point_cloud_feature.hpp>
#include <pclwrapper/point_cloud_filter.hpp>
#include <pclwrapper/point_cloud_import.hpp>
#include <pclwrapper/point_cloud_normal.hpp>
#include <pclwrapper/point_cloud_registration.hpp>

// Qt Multi-Processing
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrent>

PclProcess::PclProcess(QWidget* parent)
    : QWidget(parent), cloud_database_(XyzCloudDB::getInstance()) {}

const XyzPointCloudPropertySharedPtr& PclProcess::getXyzPointCloudProperty(const size_t id) {
  //
  return cloud_database_->queryXyzPointCloudProperty(id);
}

bool PclProcess::isPropertyExists(const size_t id, CloudProperty property) {
  auto cloud_property = getXyzPointCloudProperty(id);

  if (!cloud_property)
    return false;

  switch (property) {
    case CloudProperty::CloudPoints:
      return (cloud_property->getCloudPoints() != nullptr);
    case CloudProperty::CloudNormals:
      return (cloud_property->getCloudNormals() != nullptr);
    case CloudProperty::CloudFPFHs:
      return (cloud_property->getCloudFPFHs() != nullptr);
    case CloudProperty::CloudColor:
      return true;
    default:
      return false;
  }
}

inline auto getSearchMethodParamPair(const QString search_method, const double method_param)
    -> std::pair<pcl_wrapper::SearchMethod, pcl_wrapper::MethodParam> {
  //
  using pcl_wrapper::SearchMethod;

  if (search_method == global_config::SearchMethodList[0]) {
    return std::make_pair(SearchMethod::Radius, method_param);
  } else if (search_method == global_config::SearchMethodList[1]) {
    return std::make_pair(SearchMethod::K_neighbor, method_param);
  } else {
    return std::make_pair(SearchMethod::K_neighbor, 100);  // 默认？
  }
}

void PclProcess::loadPclFile(const std::string path,
                             const std::string path_suffix,
                             const std::string cloud_name,
                             QColor cloud_color) {
  //
  qDebug() << __func__;
  using pcl_wrapper::import::importFromPath;
  // 文件后缀校验
  using PclFileType = pcl_wrapper::import::FileType;
  PclFileType pcl_file_type = PclFileType::None;

  if (path_suffix == "ply") {
    pcl_file_type = PclFileType::PLY;
  } else if (path_suffix == "pcd") {
    pcl_file_type = PclFileType::PCD;
  } else {
    // ...
  }

  XyzCloudPointsSharedPtr xyz_src =
      importFromPath<pcl::PointXYZ>(path.c_str(), pcl_file_type);

  pcl::Indices off_nan_indices;
  xyz_src->is_dense = false;  // or removeNaNFromPointCloud won't work

  pcl::removeNaNFromPointCloud(*xyz_src, *xyz_src, off_nan_indices);

  auto cloud_id = cloud_database_->registerXyzPointCloudProperty(xyz_src, cloud_name, cloud_color);

  emit callCloudItemEditOnScreen(cloud_id, TreeList::PointCloud, ItemOperation::Add, cloud_name.c_str());
  emit callVisualizeProcess(cloud_id, CloudProperty::CloudPoints);
}

bool PclProcess::savePclFile(const size_t id,
                             const std::string abso_path,
                             const std::string suffix) {
  //
  qDebug() << __func__;
  //
  auto cloud_property = getXyzPointCloudProperty(id);

  if (!cloud_property)
    return false;
  
  qDebug() << QString::fromStdString(abso_path);

  if (suffix == "ply") {
    pcl::io::savePLYFileBinary<pcl::PointXYZ>(abso_path, *(cloud_property->getCloudPoints()));
  } else if (suffix == "pcd") {
    pcl::io::savePCDFileBinary<pcl::PointXYZ>(abso_path, *(cloud_property->getCloudPoints()));
  } else if (suffix == "vtk") {
    // pcl::io::saveVTKFile(abso_path, *(cloud_property->getTriangles()))
  } else {
    return false;
  }

  return true;
}

void PclProcess::estimateXyzCloudNormals(const std::vector<size_t> interest_cloud_id_vector,
                                         const QString search_method,
                                         const double method_param,
                                         const QVector3D viewpoint) {
  //
  qDebug() << __func__;
  using pcl_wrapper::SearchMethod;
  using pcl_wrapper::SpatialTree;
  using pcl_wrapper::normal::estimateNormals;
  using pcl_wrapper::normal::NormalEstimateResult;
  using NormalResultWatcher = QFutureWatcher<NormalEstimateResult<pcl::PointXYZ>>;
  //
  Eigen::Vector3f eigen_viewpoint = {viewpoint.x(), viewpoint.y(), viewpoint.z()};
  for (auto& each_id : interest_cloud_id_vector) {
    qDebug() << each_id << "begin";
    // 参数准备
    auto& cloud_property = cloud_database_->queryXyzPointCloudProperty(each_id);
    auto& cloud_points = cloud_property->getCloudPoints();
    auto search_method_pair = getSearchMethodParamPair(search_method, method_param);

    // 多线程运行法线估计并设置完成动作
    qDebug() << each_id << " estimate begin";

    NormalResultWatcher* result_watcher = new NormalResultWatcher(this);
    result_watcher->setFuture(QtConcurrent::run(
        estimateNormals<pcl::PointXYZ>,
        cloud_points,
        search_method_pair,
        SpatialTree::KdTree,
        eigen_viewpoint));

    connect(result_watcher, &NormalResultWatcher::finished, this, [=]() {
      qDebug() << each_id << " estimate end";
      // 将结果写入数据库
      auto& result = result_watcher->result();
      cloud_property->updateCloudPoints(result.normal_clean_cloud);
      cloud_property->updateCloudNormals(result.cloud_normal);
      // 发出可视化信号
      qDebug() << each_id << " call normal visualization";
      emit callVisualizeProcess(each_id, PclProcess::CloudProperty::CloudNormals);
      result_watcher->deleteLater();
    });

    qDebug() << __func__ << "end";
  }
}

bool PclProcess::estimateXyzCloudFPFHs(const std::vector<size_t> interest_cloud_id_vector,
                                       const QString search_method,
                                       const double method_param) {
  //
  qDebug() << __func__;
  using pcl_wrapper::feature::estimateLocalPointFeature;
  using pcl_wrapper::feature::FeatureEstimateResult;
  using FPFHResultWatcher = QFutureWatcher<FeatureEstimateResult<pcl::PointXYZ, pcl::FPFHSignature33>>;
  //
  for (auto& each_id : interest_cloud_id_vector) {
    // 参数准备
    auto& cloud_property = cloud_database_->queryXyzPointCloudProperty(each_id);
    auto& cloud_points = cloud_property->getCloudPoints();
    auto& cloud_normals = cloud_property->getCloudNormals();

    if (!cloud_normals) {  // 缺少法线
      return false;
    }

    auto search_method_pair = getSearchMethodParamPair(search_method, method_param);

    // 多线程运行特征估计并设置完成动作
    qDebug() << each_id << " estimate begin";

    FPFHResultWatcher* result_watcher = new FPFHResultWatcher(this);
    result_watcher->setFuture(QtConcurrent::run(
        estimateLocalPointFeature<pcl::PointXYZ, pcl::FPFHSignature33, pcl::FPFHEstimationOMP>,
        cloud_points,
        cloud_normals,
        search_method_pair,
        pcl_wrapper::SpatialTree::KdTree));

    connect(result_watcher, &FPFHResultWatcher::finished, this, [=]() {
      qDebug() << each_id << " estimate end";
      // 将结果写入数据库
      auto& result = result_watcher->result();
      cloud_property->updateCloudPoints(result.all_clean_cloud);
      cloud_property->updateCloudNormals(result.clean_normal);
      cloud_property->updateCloudFPFHs(result.cloud_feature);
      // 发出可视化信号
      qDebug() << each_id << " call fpfh visualization";
      emit callVisualizeProcess(each_id, PclProcess::CloudProperty::CloudFPFHs);
      result_watcher->deleteLater();
    });
  }
  qDebug() << __func__ << "end";
  return true;
}

bool PclProcess::estimateFeatureBasedRansacTransformation(const std::pair<size_t, size_t> cloud_id_pair,
                                                          const QString feature_type,
                                                          const double inlier_threshold,
                                                          const int max_iter_times) {
  //
  qDebug() << __func__;
  using pcl_wrapper::registration::estimateFeatureCorrespondences;
  using pcl_wrapper::registration::RansacEstimateResult;
  using pcl_wrapper::registration::RansacRoughTransformEstimate;
  using pcl_wrapper::registration::rejectDuplicateCorrespondences;
  using RansacResultWatcher = QFutureWatcher<RansacEstimateResult>;

  auto& src_property = cloud_database_->queryXyzPointCloudProperty(cloud_id_pair.first);
  auto& tar_property = cloud_database_->queryXyzPointCloudProperty(cloud_id_pair.second);

  // 把顺序操作包装到一个lambda函数方便后续设置FutureWatcher
  auto OperationWrapFunc = [=]() -> RansacEstimateResult {
    pcl::CorrespondencesPtr primary_corrs;
    pcl::CorrespondencesPtr one_to_one_corrs;

    if (feature_type == global_config::LocalFeatureType[0]) {  // FPFH33
      estimateFeatureCorrespondences<pcl::FPFHSignature33>(
          src_property->getCloudFPFHs(),
          tar_property->getCloudFPFHs(),
          primary_corrs);
    } else if (feature_type == global_config::LocalFeatureType[1]) {  // SHOT352
      //
    }

    if (!primary_corrs->size())
      return RansacEstimateResult();

    rejectDuplicateCorrespondences(primary_corrs, one_to_one_corrs);

    return RansacRoughTransformEstimate<pcl::PointXYZ>(
        src_property->getCloudPoints(),
        tar_property->getCloudPoints(),
        one_to_one_corrs,
        inlier_threshold,
        max_iter_times);
  };

  // 多线程运行RANSAC粗位姿估计并设置完成动作
  RansacResultWatcher* result_watcher = new RansacResultWatcher(this);

  result_watcher->setFuture(QtConcurrent::run(OperationWrapFunc));

  connect(result_watcher, &RansacResultWatcher::finished, this, [=]() {
    qDebug() << "ransac estimate end";
    auto& result = result_watcher->result();
    // 将结果生成新点云并注册/显示

    pcl::PointCloud<pcl::PointXYZ>::Ptr rough_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*(src_property->getCloudPoints()), *rough_transformed_cloud, result.rough_transform);

    std::string new_cloud_name(src_property->getCloudName() + "_RoughTo_" + tar_property->getCloudName());
    auto& new_cloud_color = src_property->getCloudColor();

    auto new_cloud_id = cloud_database_->registerXyzPointCloudProperty(rough_transformed_cloud, new_cloud_name, new_cloud_color);

    auto new_registered_property = getXyzPointCloudProperty(new_cloud_id);

    // 局部特征可以直接转换(转换有问题)
    new_registered_property->updateCloudFPFHs(src_property->getCloudFPFHs());  // 共用智能指针

    // 匹配索引保存 旧+新
    cloud_database_->registerXyzCloudPairCorrspondences(cloud_id_pair.first, cloud_id_pair.second, result.result_correspondences);
    cloud_database_->registerXyzCloudPairCorrspondences(new_cloud_id, cloud_id_pair.second, result.result_correspondences);

    // 发出表单添加及可视化信号
    qDebug() << "call rough transformed cloud visualization";
    emit callCloudItemEditOnScreen(new_cloud_id, TreeList::PointCloud, ItemOperation::Add, new_cloud_name.c_str());
    emit callVisualizeProcess(new_cloud_id, CloudProperty::CloudPoints);

    result_watcher->deleteLater();
  });

  qDebug() << __func__ << " end";
  return true;
}

void PclProcess::processVoxelGridFilter(const std::vector<size_t> interest_cloud_id_vector,
                                        const double leaf_grid_cube_size,
                                        const bool modify_on_source) {
  //
  qDebug() << __func__;
  using pcl_wrapper::filter::executeVoxelGridFilter;
  using VoxelGridResultWatcher = QFutureWatcher<void>;
  //
  for (auto& each_id : interest_cloud_id_vector) {
    auto& input_cloud_property = cloud_database_->queryXyzPointCloudProperty(each_id);

    XyzCloudPointsSharedPtr output_cloud_points;

    if (modify_on_source) {
      output_cloud_points = input_cloud_property->getCloudPoints();
    } else {
      output_cloud_points.reset(new XyzCloudPoints);
    }

    VoxelGridResultWatcher* result_watcher = new VoxelGridResultWatcher(this);

    result_watcher->setFuture(QtConcurrent::run(
        executeVoxelGridFilter<pcl::PointXYZ>,
        input_cloud_property->getCloudPoints(),
        output_cloud_points,
        static_cast<float>(leaf_grid_cube_size)));

    connect(result_watcher, &VoxelGridResultWatcher::finished, this, [=]() {
      //
      std::string output_cloud_name = input_cloud_property->getCloudName() + "_downsized";
      auto& output_cloud_color = input_cloud_property->getCloudColor();  // 传入时还是赋值

      if (!modify_on_source) {
        auto output_cloud_id = cloud_database_->registerXyzPointCloudProperty(output_cloud_points, output_cloud_name, output_cloud_color);
        emit callCloudItemEditOnScreen(output_cloud_id, TreeList::PointCloud, ItemOperation::Add, output_cloud_name.c_str());
        emit callVisualizeProcess(output_cloud_id, CloudProperty::CloudPoints);
      } else {
        editCloudItemOnDatabase(each_id, ItemOperation::Rename, output_cloud_name.c_str());
        emit callCloudItemEditOnScreen(each_id, TreeList::PointCloud, ItemOperation::Rename, output_cloud_name.c_str());
        emit callVisualizeProcess(each_id, CloudProperty::CloudPoints, Visibility::Hide);  // Remove old
        emit callVisualizeProcess(each_id, CloudProperty::CloudNormals, Visibility::Hide);
        // 原有特征将失效(need test)
        input_cloud_property->updateCloudNormals(nullptr);
        input_cloud_property->updateCloudFPFHs(nullptr);
        // 原有特征匹配点对也将失效
        cloud_database_->deleteXyzCloudPairCorrespondences(each_id);
        //
        emit callVisualizeProcess(each_id, CloudProperty::CloudPoints);  // Add new

        // // 先更改界面才能更改数据库，因为更改界面过程中会需要调用数据库
        // pcl_database_->modifyXyzPointCloudNameIndex(each_id.toStdString(), output_cloud_name);
      }
      result_watcher->deleteLater();
    });
  }
}

bool PclProcess::mergeCloudPoints(const std::vector<size_t> interest_cloud_id_vector, QColor output_color) {
  qDebug() << __func__;
  //

  if (interest_cloud_id_vector.size() < 2) {
    return false;
  }

  std::vector<XyzCloudPointsSharedPtr> point_cloud_vector;
  std::string name_list;
  XyzCloudPointsSharedPtr tmp_in;
  XyzCloudPointsSharedPtr tmp_out;

  for (auto& each_id : interest_cloud_id_vector) {
    point_cloud_vector.push_back(getXyzPointCloudProperty(each_id)->getCloudPoints());
    name_list += getXyzPointCloudProperty(each_id)->getCloudName() + " ";
  }

  tmp_in = point_cloud_vector[0];
  tmp_out.reset(new XyzCloudPoints);

  for (size_t index = 1; index < interest_cloud_id_vector.size(); ++index) {
    pcl::concatenate<pcl::PointXYZ>(*tmp_in, *(point_cloud_vector[index]), *tmp_out);
    tmp_in = tmp_out;
  }

  auto& output_cloud_points = tmp_out;
  std::string output_cloud_name = "merge_" + name_list;

  auto output_cloud_id = cloud_database_->registerXyzPointCloudProperty(output_cloud_points, output_cloud_name, output_color);
  emit callCloudItemEditOnScreen(output_cloud_id, TreeList::PointCloud, ItemOperation::Add, output_cloud_name.c_str());
  emit callVisualizeProcess(output_cloud_id, CloudProperty::CloudPoints);

  return true;
}

void PclProcess::editCloudItemOnDatabase(size_t cloud_id, ItemOperation op, QString new_name) {
  qDebug() << __func__;

  // cloud_database_->queryXyzPointCloudProperty(cloud_id)->updateCloudColor(cloud_database_->queryXyzPointCloudProperty(cloud_id)->getCloudColor());

  if (op == ItemOperation::Remove) {
    cloud_database_->deleteXyzPointCloudProperty(cloud_id);
    cloud_database_->deleteXyzCloudPairCorrespondences(cloud_id);
  } else if (op == ItemOperation::Rename) {
    cloud_database_->queryXyzPointCloudProperty(cloud_id)->updateCloudName(new_name.toStdString());
  } else {
    // ...
  }
}
