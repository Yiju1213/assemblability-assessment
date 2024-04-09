# CMake Requirement
cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

if(WIN32)
    # add_compile_options(/W4)
    add_compile_options(/wd4819)
endif()

# Project Name
project(YIJU_GraduationProject
	  VERSION 1.0.0
	  LANGUAGES CXX
)

# C++ Requirement
set(CMAKE_CXX_STANDARD 11)
#   指定是否使用编译器特定的扩展
set(CMAKE_CXX_EXTENSIONS OFF)
#   是否强制要求C++标准为${CMAKE_CXX_STANDARD},如果为OFF则可能使用旧的标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# specify the C++ standard
add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_11)

# add compiler warning flags just when building this project via
# the BUILD_INTERFACE genex
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
  "$<$<CXX_COMPILER_ID:MSVC>:/bigobj>"
)

# 默认编译动态库（暂时不要）
# set(BUILD_SHARED_LIBS ON)

# 编译位置无关的代码（动态库用）
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 开启项目分组
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Qt相关设置
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Qt模块检索
set(Qt5_DIR "E:/Qt/Qt5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5")
find_package(Qt5 COMPONENTS Widgets Concurrent REQUIRED)

if(Qt5_FOUND)
  message(STATUS "Found Qt5: ${Qt5_DIR}")
else()
  set(Qt5_DIR "Qt5_DIR-NOTFOUND" CACHE PATH "Qt5_DIR")
endif()

# PCL模块检索
set(PCL_DIR "C:/Program Files/PCL1.13.0-VTK9.0.1/cmake")
find_package(PCL 1.13.0 REQUIRED)
find_package(OpenMP REQUIRED)

# VTK(9.0.1)模块检索
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(VTK REQUIRED)

# FCL模块检索
set(fcl_DIR "C:/Program Files/FCL/install/cmake")
find_package(fcl)

# 查找自定义cmake时要设置CMAKE_MODULE_PATH
# CMAKE_MODULE_PATH 会被find_package优先搜索，搜索Findxxx.cmake文件
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
# PCLWrapper自定义模块检索
find_package(PCLWrapper REQUIRED)

# 源码模块检索
#
add_subdirectory(main)
add_subdirectory(GlobalConfig)
# 数据管理层
add_subdirectory(PropertyDataBase)
# 数据处理层
add_subdirectory(MeshProcess)
add_subdirectory(PointCloudProcess)
add_subdirectory(CollideDetect)
# 可视化交互层
add_subdirectory(SubDialog)
add_subdirectory(SubWidget)
add_subdirectory(MainWindow)


