include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB _h_header   "*.h")
file(GLOB _hpp_header "*.hpp")
file(GLOB _source     "*.cpp")

add_library(MeshProcess
    ${_h_header}
    ${_hpp_header}
    ${_source}
)

target_link_libraries(MeshProcess PUBLIC
    GlobalConfig
    PropertyDataBase
    Qt5::Concurrent
    )