cmake_minimum_required(VERSION 3.21)
project(controller)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#include(GNUInstallDirs)
set(PACKAGE_NAME controller)


# Library target
add_library(controller INTERFACE)


# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(controller INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(controller INTERFACE
            Python3::NumPy
            )
else()
    target_compile_definitions(controller INTERFACE WITHOUT_NUMPY)
endif()
install(
        TARGETS controller
        EXPORT install_targets
)

add_executable(controller_main controller/controller_main.cpp controller/KinematicModel.cpp)
target_link_libraries(controller_main PRIVATE controller)

add_executable(trajecotry_main trajectory/main.cpp)
target_link_libraries(trajecotry_main PRIVATE controller)


