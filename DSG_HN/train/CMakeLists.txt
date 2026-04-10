cmake_minimum_required(VERSION 3.12)
project(dsg_train)

# cmake .. \ -DCMAKE_C_COMPILER=gcc-11 \ -DCMAKE_CXX_COMPILER=g++-11 \ -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \ -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11

# - include toolkit for cuda path
# - additionally, if your gcc version is 12 by default and you are using cuda toolkit 11.8, need to use gcc 11

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DSPDLOG_FMT_EXTERNAL)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/mujoco")
set(POLICY_CONFIG_DIR "${PROJECT_SOURCE_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1")
set(JOYSTICK_SRC "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/src")
set(LIBTORCH_PATH "$ENV{HOME}/miniconda3/lib/python3.13/site-packages/torch")

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

find_package(Torch REQUIRED)


# DO NOT add $ENV{CONDA_PREFIX}/include here globally — conda ships fmt v11
# which conflicts with system spdlog (expects fmt v9). Only targets that need
# conda headers (train, test — for svm.h) get it via target_include_directories.
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
  ${JOYSTICK_SRC}
)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config/scene)
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

# sandbox_mpc is split into two TUs to avoid the spdlog/fmt vs libtorch/fmt clash:
#   sandbox_mpc.cpp       — main loop, robot bridge, spdlog (NO torch headers)
#   sandbox_mpc_torch.cpp — model, MPC, normaliser   (torch headers, NO spdlog)
#
# Bridge objects (robot_bridge, joystick, mujoco_engine) also never see torch headers.
add_library(sandbox_mpc_bridge OBJECT
  sandbox_mpc.cpp
  ${JOYSTICK_SRC}/joystick/joystick.cc
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)

add_library(sandbox_mpc_torch_obj OBJECT sandbox_mpc_torch.cpp)
target_include_directories(sandbox_mpc_torch_obj PRIVATE ${TORCH_INCLUDE_DIRS})

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt spdlog
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw
  ${TORCH_LIBRARIES}
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_executable(train 
  train.cpp
  agent.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)

add_executable(test 
  test.cpp
  agent.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)

if(DEFINED ENV{CONDA_PREFIX})
  target_include_directories(train PRIVATE $ENV{CONDA_PREFIX}/include)
  target_include_directories(test PRIVATE $ENV{CONDA_PREFIX}/include)
  target_link_directories(train PRIVATE $ENV{CONDA_PREFIX}/lib)
  target_link_directories(test PRIVATE $ENV{CONDA_PREFIX}/lib)
  set(_LIBSVM_SO "$ENV{CONDA_PREFIX}/lib/libsvm.so")
  if(EXISTS "${_LIBSVM_SO}")
    target_link_libraries(train PRIVATE "${_LIBSVM_SO}")
    target_link_libraries(test PRIVATE "${_LIBSVM_SO}")
  endif()
endif()

add_executable(sandbox_mpc
  $<TARGET_OBJECTS:sandbox_mpc_bridge>
  $<TARGET_OBJECTS:sandbox_mpc_torch_obj>
)

add_executable(transition_train_transformer_delta
  transition_train_transformer_delta.cpp
)
target_link_libraries(transition_train_transformer_delta PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_rollout_eval_transformer_delta
  transition_rollout_eval_transformer_delta.cpp
)
target_link_libraries(transition_rollout_eval_transformer_delta PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)
