include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB _h_header   "*.h")
file(GLOB _hpp_header "*.hpp")
file(GLOB _source     "*.cpp")

# PCL include VTK itself

add_library(PropertyDataBase
    ${_h_header}
    ${_hpp_header}
    ${_source})

target_include_directories(PropertyDataBase PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${PCLWrapper_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_link_directories(PropertyDataBase PUBLIC
    ${PCL_LIBRARY_DIRS})

target_link_libraries(PropertyDataBase PUBLIC
    GlobalConfig
    ${PCL_LIBRARIES})