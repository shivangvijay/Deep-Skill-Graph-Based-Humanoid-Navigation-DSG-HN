cmake_minimum_required(VERSION 3.12)
project(dsg_train)

# cmake .. \ -DCMAKE_C_COMPILER=gcc-11 \ -DCMAKE_CXX_COMPILER=g++-11 \ -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \ -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11

# - include toolkit for cuda path
# - additionally, if your gcc version is 12 by default and you are using cuda toolkit 11.8, need to use gcc 11

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/mujoco")
set(POLICY_CONFIG_DIR "${PROJECT_SOURCE_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1")
set(LIBTORCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libtorch")

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

find_package(Torch REQUIRED)


include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../deploy/include
  ${POLICY_CONFIG_DIR}
  ${MUJOCO_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../unitree_sdk2/include
  ${PROJECT_SOURCE_DIR}/../robot_bridge/include

)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw
  ${TORCH_LIBRARIES}
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_executable(example 
  example.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)