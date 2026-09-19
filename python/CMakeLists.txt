cmake_minimum_required(VERSION 3.16.3)

find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
    set(ARX5_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(ARX5_YAML_CPP_TARGET yaml-cpp)
endif()

find_path(ARX5_YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h REQUIRED)
include_directories(${ARX5_YAML_CPP_INCLUDE_DIR})
set(ARX5_YAML_CPP_INCLUDE_FLAG "-I${ARX5_YAML_CPP_INCLUDE_DIR}")
add_compile_options(${ARX5_YAML_CPP_INCLUDE_FLAG})

pybind11_add_module(arx5_interface
    arx5_pybind.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/app/joint_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/app/cartesian_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/app/controller_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/app/config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/utils.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/libcan)
add_compile_options(-pthread)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/x86_64)
endif()

target_link_libraries(arx5_interface PRIVATE
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    spdlog::spdlog
    Eigen3::Eigen
    Threads::Threads
    kdl_parser
    orocos-kdl
    pthread
    soem
    ${ARX5_YAML_CPP_TARGET}
)
target_include_directories(arx5_interface PUBLIC ${EIGEN3_INCLUDE_DIRS})

# Hack for py310
target_include_directories(arx5_interface PUBLIC $ENV{CONDA_PREFIX}/include/kdl_parser $ENV{CONDA_PREFIX}/include/urdfdom_headers)

# # Optional: set the output directory for the built module
set_target_properties(arx5_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(arx5_interface PROPERTIES PREFIX "")
set_target_properties(arx5_interface PROPERTIES OUTPUT_NAME "arx5_interface")
