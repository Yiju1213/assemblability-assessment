#include "global_config.h"


#include <QFont>
#include <QStringList>

namespace global_config {
  // 不允许更改顺序
  QStringList SpatialStructList = {"KdTree", "OctTree"};
  QStringList SearchMethodList = {"Radius Search", "K Neighbor Search"};
  QStringList LocalFeatureType = {"FPFH33", "SHOT352"};

  // 字体设置
  QFont TitleLevel1("Times New Roman", 14, QFont::Weight::DemiBold);
  QFont TitleLevel2("Times New Roman", 12, QFont::Weight::Bold);
  QFont Label("Times New Roman", 10, QFont::Weight::Normal);

  //
  // Property CloudProperty;

}  // namespace global_config