cmake_minimum_required(VERSION 3.12)
project(dsg_train)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(TRAIN_DIR "${PROJECT_SOURCE_DIR}/..")
set(MUJOCO_DIR "$ENV{CONDA_PREFIX}")
set(POLICY_CONFIG_DIR "${TRAIN_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/unitree_robots/g1")
set(LIBTORCH_PATH "${TRAIN_DIR}/thirdparty/libtorch")

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

find_package(Torch REQUIRED)

include_directories(
  $ENV{CONDA_PREFIX}/include/eigen3
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include/ddscxx
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include
  ${TRAIN_DIR}/include
  ${TRAIN_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/include
  ${TRAIN_DIR}/../deploy/include
  ${POLICY_CONFIG_DIR}
  ${MUJOCO_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/include   # headers only — IDL types used as plain structs
  ${TRAIN_DIR}/../robot_bridge/include
)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${TRAIN_DIR}/config)

set(COMMON_SOURCES
  ${TRAIN_DIR}/agent.cpp
  ${TRAIN_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${TRAIN_DIR}/../robot_bridge/robot_bridge.cpp
  ${TRAIN_DIR}/../robot_bridge/robot_bridge_train.cpp
)

set(COMMON_LIBS
  pthread
  $ENV{CONDA_PREFIX}/lib/libboost_program_options.dylib
  $ENV{CONDA_PREFIX}/lib/libyaml-cpp.dylib
  $ENV{CONDA_PREFIX}/lib/libfmt.dylib
  ${TRAIN_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/lib/libonnxruntime.dylib
  ${MUJOCO_DIR}/lib/libmujoco.dylib
  $ENV{CONDA_PREFIX}/lib/libglfw.dylib
  ${TORCH_LIBRARIES}
)

add_executable(train ${TRAIN_DIR}/train.cpp ${COMMON_SOURCES})
target_link_libraries(train PRIVATE ${COMMON_LIBS})

add_executable(test ${TRAIN_DIR}/test.cpp ${COMMON_SOURCES})
target_link_libraries(test PRIVATE ${COMMON_LIBS})
