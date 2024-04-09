# FindPCLWrapper
# --------
# Author: Yiju
# Brife: Find the PCL Wrapper 
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# The following variables will be defined:
#
# ``PCLWrapper_FOUND`` True if PCLWrapper found on the local system
#
# ``PCLWrapper_DIRS`` Location of PCLWrapper(root dir)
#
# ``PCLWrapper_INCLUDE_DIRS`` Location of PCLWrapper header files
#
# ``PCLWrapper_LIBRARY_DIRS`` Location of PCLWrapper libraries
#
# ``PCLWrapper_LIBRARIES`` List of the PCLWrapper libraries found
#
# ``PCLWrapper_EXECUTABLE`` Location of PCLWrapper program
#

# 防止重复引入
if(YIJU_PCLWrapper_ALREADY_INCLUDED)
  message("YIJU_PCLWrapper_ALREADY_INCLUDED!")
	return()
endif()
set(YIJU_PCLWrapper_ALREADY_INCLUDED 1)

# 版本号-暂时不需要

# find_path 搜索包含某个文件的路径
# 如果在某个路径下发现了该文件，该结果会被存储到该变量中；如果没有找到，存储的结果将会是<VAR>-NOTFOUND

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # 未修改
	# set(_pclwrapper_h_dir "include/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}m")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(_pclwrapper_h_dir "include")
endif()

find_path(PCLWrapper_DIRS # /PCLWrapper
	NAMES 
		"${_pclwrapper_h_dir}/pclwrapper"
	PATHS 
		${CMAKE_SOURCE_DIR}/../../PCL/PCLWrapper
	NO_SYSTEM_ENVIRONMENT_PATH
	NO_CMAKE_SYSTEM_PATH
)

if(PCLWrapper_DIRS)
  message("PCLWRapper_DIRS Found in ${PCLWrapper_DIRS}")
else()
  message("Error: PCLWrapper_DIRS Not Found")
  return()
endif()

find_path(PCLWrapper_INCLUDE_DIRS # /PCLWrapper/include
	NAMES
		"pclwrapper/PCLWrapper.hpp"
	HINTS
		${PCLWrapper_DIRS}/${_pclwrapper_h_dir}
)

if(PCLWrapper_INCLUDE_DIRS)
  message("PCLWrapper_INCLUDE_DIRS Found in ${PCLWrapper_INCLUDE_DIRS}")
else()
  message("Error: PCLWrapper_INCLUDE_DIRS Not Found")
  return()
endif()

# 没有可执行项
# if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
# 	set(Python_EXECUTABLE "${Python_DIRS}/python.exe")
# else()
# 	set(Python_EXECUTABLE "${Python_DIRS}/bin/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}")
# endif()

# 没有静态库
# set(Python_VERSION "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}.${Python_VERSION_PATCH}")
# find_path(Python_LIBRARY_DIRS
# 	NAMES
# 		python${Python_VERSION_MAJOR}${Python_VERSION_MINOR}.lib libpython${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}m.so
# 	HINTS
# 		${Python_DIRS}/libs ${Python_DIRS}/lib
# )

# 在父级项目中直接添加库？
# set(Python_LIBRARIES FASTCAE::PYTHON)
# add_library(FASTCAE::PYTHON SHARED IMPORTED)
# 在父级项目中直接添加库的头文件依赖目录？
# set_property(TARGET FASTCAE::PYTHON PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Python_INCLUDE_DIRS})
# 没看懂啥意思
# set_property(TARGET FASTCAE::PYTHON APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)

# 添加库文件属性？
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	# add_library(FASTCAE::FFI SHARED IMPORTED)
	# set_property(TARGET FASTCAE::FFI APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
	# set_target_properties(FASTCAE::FFI PROPERTIES
	# 	IMPORTED_LOCATION_RELEASE "${Python_LIBRARY_DIRS}/libffi.so.8.1.0"
	# 	IMPORTED_SONAME_RELEASE "libffi.so.8"
	# )
	# set_target_properties(FASTCAE::PYTHON PROPERTIES
	# 	IMPORTED_LOCATION_RELEASE "${Python_LIBRARY_DIRS}/libpython${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}m.so.1.0"
	# 	IMPORTED_SONAME_RELEASE "libpython${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}m.so"
	# )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	# set_target_properties(FASTCAE::PYTHON PROPERTIES
	# 	IMPORTED_IMPLIB_RELEASE "${Python_LIBRARY_DIRS}/python${Python_VERSION_MAJOR}${Python_VERSION_MINOR}.lib"
	# 	IMPORTED_LOCATION_RELEASE "${Python_DIRS}/python${Python_VERSION_MAJOR}${Python_VERSION_MINOR}.dll"
	# )
endif()


include(FindPackageHandleStandardArgs)
# 如果找到所有需要的变量，并且版本匹配，则将PCLWrapper_FOUND变量设置为TRUE
find_package_handle_standard_args(PCLWrapper
	FOUND_VAR
    PCLWrapper_FOUND
	REQUIRED_VARS
		PCLWrapper_DIRS
		PCLWrapper_INCLUDE_DIRS
		# PCLWrapper_LIBRARY_DIRS
		# PCLWrapper_LIBRARIES
		# PCLWrapper_EXECUTABLE
	# VERSION_VAR
  # PCLWrapper_VERSION
)