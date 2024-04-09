include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB _h_header   "*.h")
file(GLOB _hpp_header "*.hpp")
file(GLOB _source     "*.cpp")

# 资源文件要在executable加才有用
set(_qrc "${CMAKE_CURRENT_SOURCE_DIR}/../qrc/tool-icon.qrc")
qt5_add_resources(_resource ${_qrc})

message("qrc ${_resource}")

add_executable(${PROJECT_NAME}
    ${_h_header}
    ${_hpp_header}
    ${_source}
    ${_resource}
)

# 链接编译标志
target_link_libraries(${PROJECT_NAME} PUBLIC
      compiler_flags
      MainWindow)