cmake_minimum_required(VERSION 3.12)
project(dsg_train)

# cmake .. \ -DCMAKE_C_COMPILER=gcc-11 \ -DCMAKE_CXX_COMPILER=g++-11 \ -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \ -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/gcc-11

# - include toolkit for cuda path
# - additionally, if your gcc version is 12 by default and you are using cuda toolkit 11.8, need to use gcc 11

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DSPDLOG_FMT_EXTERNAL)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR    "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/mujoco")
set(JOYSTICK_SRC  "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/src")
set(POLICY_CONFIG_DIR "${PROJECT_SOURCE_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1")
set(LIBTORCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libtorch")
# set(LIBTORCH_PATH "${PROJECT_SOURCE_DIR}/../../../_/DSG_HN/libtorch") Keeping old definition for consistency, may need to change this path depending on libtorch location

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES "70;75;80;86")
endif()

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

find_package(Torch REQUIRED)


include_directories(
  $ENV{CONDA_PREFIX}/include
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

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config/scene)
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

link_directories($ENV{CONDA_PREFIX}/lib)
add_executable(transition_train_gaussian
  transition_train_gaussian.cpp
)

target_link_libraries(transition_train_gaussian PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_train_gaussian_delta
  transition_train_gaussian_delta.cpp
)

target_link_libraries(transition_train_gaussian_delta PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_infer
  transition_infer.cpp
)

target_link_libraries(transition_infer PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_rollout_eval
  transition_rollout_eval.cpp
)

target_link_libraries(transition_rollout_eval PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_rollout_eval_delta
  transition_rollout_eval_delta.cpp
)

target_link_libraries(transition_rollout_eval_delta PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

add_executable(transition_mpc_eval
  transition_mpc_eval.cpp
)

target_link_libraries(transition_mpc_eval PRIVATE
  ${TORCH_LIBRARIES}
  libboost_program_options.a
  fmt
)

# sandbox_mpc: split into two object libs to avoid the fmt/libtorch ODR clash.
#   sandbox_mpc.cpp       — robot bridge + main loop  (no torch headers)
#   sandbox_mpc_torch.cpp — model, MPC planner         (torch headers only)
add_library(sandbox_mpc_bridge OBJECT
  sandbox_mpc.cpp
  ${JOYSTICK_SRC}/joystick/joystick.cc
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)
target_include_directories(sandbox_mpc_bridge PRIVATE ${JOYSTICK_SRC})

add_library(sandbox_mpc_torch_obj OBJECT sandbox_mpc_torch.cpp)
target_include_directories(sandbox_mpc_torch_obj PRIVATE ${TORCH_INCLUDE_DIRS})

add_executable(sandbox_mpc
  $<TARGET_OBJECTS:sandbox_mpc_bridge>
  $<TARGET_OBJECTS:sandbox_mpc_torch_obj>
)
target_link_libraries(sandbox_mpc PRIVATE
  ${TORCH_LIBRARIES}
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  $ENV{CONDA_PREFIX}/lib/libsvm.so
)

add_executable(test_mpc_obstacles
  test_mpc_obstacles.cpp
  sandbox_mpc_torch.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
)
target_link_libraries(test_mpc_obstacles PRIVATE
  ${TORCH_LIBRARIES}
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt spdlog
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw
  $ENV{CONDA_PREFIX}/lib/libsvm.so
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

add_executable(dsc
  dsc.cpp
  agent.cpp
  skill.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)

add_executable(dsg
  dsg.cpp
  dsc.cpp
  agent.cpp
  skill.cpp
  sandbox_mpc_torch.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../robot_bridge/robot_bridge_train.cpp
)
target_compile_definitions(dsg PRIVATE DSG_BUILD)