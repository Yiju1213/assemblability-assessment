include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB _h_header   "*.h")
file(GLOB _hpp_header "*.hpp")
file(GLOB _source     "*.cpp")
file(GLOB _ui         "*.ui")
qt5_wrap_ui(_interface ${_ui})

add_library(SubDialog
    ${_h_header}
    ${_hpp_header}
    ${_source}
    ${_interface}
)

target_link_libraries(SubDialog PUBLIC 
    compiler_flags
    Qt5::Widgets
    GlobalConfig
    )


