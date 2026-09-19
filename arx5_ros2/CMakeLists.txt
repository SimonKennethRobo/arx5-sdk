cmake_minimum_required(VERSION 3.16.3)
project(arx5_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Threads REQUIRED)
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

# soem (EtherCAT master) has no CMake package config and, unlike the SDK's
# other cmake deps, is not available as a ROS/apt package; it is only shipped
# through the SDK's conda environments (conda_environments/*.yaml). Its
# directory can be overridden with -DARX5_SOEM_DIR=... if soem is installed
# elsewhere; it must NOT be resolved by activating conda (i.e. by pointing
# CMAKE_PREFIX_PATH at the conda env), since that would also pull in a newer
# fmt/spdlog ABI than the one ROS 2 Foxy's own libraries were built against,
# and break linking against rclcpp.
if(NOT DEFINED ARX5_SOEM_DIR AND DEFINED ENV{CONDA_PREFIX})
  set(ARX5_SOEM_DIR "$ENV{CONDA_PREFIX}/lib")
endif()
find_library(ARX5_SOEM_LIBRARY NAMES soem HINTS ${ARX5_SOEM_DIR})
if(NOT ARX5_SOEM_LIBRARY)
  message(FATAL_ERROR
    "libsoem not found. Pass -DARX5_SOEM_DIR=<dir containing libsoem> "
    "(e.g. the lib dir of the SDK's conda env, see ../conda_environments); "
    "do not `conda activate` before building, it conflicts with ROS 2's own "
    "fmt/spdlog.")
endif()

# The prebuilt lib/aarch64/{libhardware,libsolver}.so shipped in the SDK are
# built with a newer GCC than ships with Ubuntu 20.04 (Foxy's target OS), so
# they need a libstdc++.so.6 newer than the system one (missing symbols such
# as GLIBCXX_3.4.29). The conda env's libstdc++ satisfies this and, being
# strictly backwards-compatible, is also safe to use for the rest of the
# process (including ROS 2's own libraries, which only need older symbols).
find_library(ARX5_LIBSTDCXX NAMES stdc++.so.6 stdc++ HINTS ${ARX5_SOEM_DIR})
if(NOT ARX5_LIBSTDCXX)
  message(WARNING
    "A newer libstdc++ (matching the conda env used to build libhardware/"
    "libsolver) was not found next to libsoem; linking may fail with "
    "GLIBCXX version errors.")
endif()

# Path to the arx5-sdk checkout this package wraps. Defaults to the parent of
# this directory (i.e. this package is expected to live at <arx5-sdk>/ros2),
# but can be overridden with -DARX5_SDK_DIR=... if the package is relocated.
if(NOT DEFINED ARX5_SDK_DIR)
  set(ARX5_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()
get_filename_component(ARX5_SDK_DIR "${ARX5_SDK_DIR}" ABSOLUTE)

if(NOT EXISTS "${ARX5_SDK_DIR}/include/app/cartesian_controller.h")
  message(FATAL_ERROR "ARX5_SDK_DIR (${ARX5_SDK_DIR}) does not look like an arx5-sdk checkout")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/x86_64)
else()
  message(FATAL_ERROR "Unsupported CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# utils.cpp requires SDK_ROOT at compile time; the node itself resolves the
# URDF path from ARX5_SDK_ROOT_DIR directly, so this is only used as a
# fallback inside the SDK.
add_compile_definitions(SDK_ROOT="${ARX5_SDK_DIR}")
add_compile_definitions(ARX5_SDK_ROOT_DIR="${ARX5_SDK_DIR}")

add_executable(arx5_ros2_node
  src/arx5_ros2_node.cpp
  ${ARX5_SDK_DIR}/src/app/cartesian_controller.cpp
  ${ARX5_SDK_DIR}/src/app/joint_controller.cpp
  ${ARX5_SDK_DIR}/src/app/controller_base.cpp
  ${ARX5_SDK_DIR}/src/app/config.cpp
  ${ARX5_SDK_DIR}/src/utils.cpp
)

target_include_directories(arx5_ros2_node PRIVATE
  include
  ${ARX5_SDK_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

ament_target_dependencies(arx5_ros2_node
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_msgs
  std_srvs
)

target_link_libraries(arx5_ros2_node
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
  Eigen3::Eigen
  Threads::Threads
  spdlog::spdlog
  fmt::fmt
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${ARX5_SOEM_LIBRARY}
  ${ARX5_LIBSTDCXX}
  ${Boost_LIBRARIES}
  ${ARX5_YAML_CPP_TARGET}
)

if(ARX5_LIBSTDCXX)
  # The conda env's lib dir must be searched LAST: it also contains its own
  # copies of orocos-kdl/kdl_parser/urdf (pulled in by the SDK's conda
  # environment) which are ABI-incompatible with ROS 2 Foxy's own build of
  # those same sonames. Putting ROS's lib dir first ensures those resolve
  # correctly there, while libstdc++.so.6 (absent/older in ROS's and the
  # system's lib dirs) still falls through to the conda copy.
  get_filename_component(ARX5_LIBSTDCXX_DIR "${ARX5_LIBSTDCXX}" DIRECTORY)
  get_filename_component(ARX5_ROS_LIB_DIR "${kdl_parser_LIBRARIES}" DIRECTORY)
  set_target_properties(arx5_ros2_node PROPERTIES
    BUILD_RPATH "${ARX5_LIB_DIR};${ARX5_ROS_LIB_DIR};${ARX5_LIBSTDCXX_DIR}"
    INSTALL_RPATH "${ARX5_LIB_DIR};${ARX5_ROS_LIB_DIR};${ARX5_LIBSTDCXX_DIR}"
  )
endif()

install(TARGETS arx5_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY ${ARX5_SDK_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY ${ARX5_SDK_DIR}/models/
  DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
