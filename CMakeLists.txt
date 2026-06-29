cmake_minimum_required(VERSION 3.14)

project(CartPoleSimulator VERSION 1.0 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_executable(simulation 
    src/cart_pole.cpp
    src/lqr_controller.cpp
    src/simulator.cpp
    src/main.cpp
)
# Ajoute l'option -O3 uniquement à la cible "simulation"
target_compile_options(simulation PRIVATE -O3)