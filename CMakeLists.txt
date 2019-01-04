cmake_minimum_required(VERSION 2.8)
project(VoxelMultiAxisAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(qhullDir ${CMAKE_CURRENT_SOURCE_DIR}/qhull)
INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/src)
LINK_DIRECTORIES(${qhullDir}/build)
SET(qhullLibs ${qhullDir}/build/libqhullcpp.a libqhullstatic_r.a)

option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui" ON)

find_package(LIBIGL REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)


include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS})

file(GLOB SRCFILES
        src/main.cpp
        src/bsp_tree.cpp
        src/voxel.cpp
        src/file_parser.cpp
        src/kdtree.cpp
        src/trimming.cpp
        src/bst.cpp
        src/dual_contouring.cpp
    )

add_executable(${PROJECT_NAME}_bin ${SRCFILES})
target_link_libraries(
        ${PROJECT_NAME}_bin
        ${qhullLibs}
        ${Boost_FILESYSTEM_LIBRARY}
        ${EIGEN3_LIBRARIES}
        igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::cgal)