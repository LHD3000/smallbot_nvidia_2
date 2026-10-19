cmake_minimum_required(VERSION 3.0)
project(experiment)

set(CMAKE_CXX_STANDARD 20)

find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(experiment_tools SHARED
    src/ui/viewer_2d.cpp 
    src/ui/figure.cpp 
    src/ui/workbench.cpp
    src/ui/args_inspector.cpp
    src/ui/loader.cpp
    src/ui/plant_inspector.cpp
    src/ui/controller_inspector.cpp
    src/ui/simulator_controller.cpp
    src/ui/recorder.cpp
    src/lib_loader.cpp
    src/database.cpp
    src/simulator.cpp
    )
target_include_directories(experiment_tools PUBLIC include)
target_link_libraries(experiment_tools PUBLIC Qt5::Widgets Qt5::Charts)

add_executable(workbench src/app.cpp)
target_link_libraries(workbench PUBLIC experiment_tools)

add_library(demo_pack SHARED src/simpack/demo_pack.cpp)
target_link_libraries(demo_pack PUBLIC experiment_tools control_algorithm)

add_executable(figure_viewer test/figure.cpp)
target_link_libraries(figure_viewer PUBLIC experiment_tools)