include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB _h_header   "*.h")
file(GLOB _hpp_header "*.hpp")
file(GLOB _source     "*.cpp")

add_library(CollideDetect
    ${_h_header}
    ${_hpp_header}
    ${_source}
)

# 链接编译标志
target_include_directories(CollideDetect PUBLIC
    ${VTK_INCLUDE_DIRS})
    
target_link_directories(CollideDetect PUBLIC
    ${VTK_LIBRARY_DIRS})

target_link_libraries(CollideDetect PUBLIC
    GlobalConfig
    compiler_flags
    Qt5::Concurrent
    ${VTK_LIBRARIES}
    ${FCL_LIBRARIES}
    )


