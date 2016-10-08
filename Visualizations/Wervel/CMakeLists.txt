
cmake_minimum_required(VERSION 2.8)

PROJECT(WervelVisualization)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(WervelEx MACOSX_BUNDLE Wervel)

add_definitions(-std=c++11)

if(VTK_LIBRARIES)
  target_link_libraries(WervelEx ${VTK_LIBRARIES})
else()
  target_link_libraries(WervelEx vtkHybrid vtkWidgets)
endif()
