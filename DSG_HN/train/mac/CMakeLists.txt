cmake_minimum_required(VERSION 3.12)
project(dsg_train)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(TRAIN_DIR "${PROJECT_SOURCE_DIR}/..")
set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.3.6")
set(POLICY_CONFIG_DIR "${TRAIN_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/unitree_robots/g1")
set(LIBTORCH_PATH "${TRAIN_DIR}/thirdparty/libtorch")

list(APPEND CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")

find_package(Torch REQUIRED)

set(THIRDPARTY_HINTS
  $ENV{CONDA_PREFIX}
  /opt/homebrew
  /usr/local
)

find_path(GLFW_INCLUDE_DIR
  NAMES GLFW/glfw3.h
  HINTS ${THIRDPARTY_HINTS}
  PATH_SUFFIXES include
)
find_library(GLFW_LIBRARY
  NAMES glfw glfw3 libglfw.dylib
  HINTS ${THIRDPARTY_HINTS}
  PATH_SUFFIXES lib
)
if(NOT GLFW_INCLUDE_DIR OR NOT GLFW_LIBRARY)
  message(FATAL_ERROR "GLFW not found. Install glfw in your active env/system and reconfigure CMake.")
endif()

find_path(SPDLOG_INCLUDE_DIR
  NAMES spdlog/spdlog.h
  HINTS ${THIRDPARTY_HINTS}
  PATH_SUFFIXES include
)

include_directories(
  /opt/homebrew/include
  $ENV{CONDA_PREFIX}/include
  $ENV{CONDA_PREFIX}/include/eigen3
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include/ddscxx
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include
  ${TRAIN_DIR}/include
  ${TRAIN_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/include
  ${TRAIN_DIR}/../deploy/include
  ${POLICY_CONFIG_DIR}
  ${MUJOCO_DIR}
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/include   # headers only — IDL types used as plain structs
  ${TRAIN_DIR}/../robot_bridge/include
  ${GLFW_INCLUDE_DIR}
)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config/scene)
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

# Symlink mac/models -> mac/build so that test can load models via "../models/"
file(CREATE_LINK ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/models SYMBOLIC)

set(COMMON_SOURCES
  ${TRAIN_DIR}/agent.cpp
  ${TRAIN_DIR}/skill.cpp
  ${TRAIN_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${TRAIN_DIR}/../robot_bridge/robot_bridge.cpp
  ${TRAIN_DIR}/../robot_bridge/robot_bridge_train.cpp
)

set(COMMON_LIBS
  pthread
  /opt/homebrew/lib/libboost_program_options.dylib
  $ENV{CONDA_PREFIX}/lib/libyaml-cpp.dylib
  $ENV{CONDA_PREFIX}/lib/libfmt.dylib
  ${TRAIN_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/lib/libonnxruntime.dylib
  ${MUJOCO_DIR}/lib/libmujoco.3.3.6.dylib
  ${GLFW_LIBRARY}
  $ENV{CONDA_PREFIX}/lib/libsvm.dylib
  ${TORCH_LIBRARIES}
)

if(SPDLOG_INCLUDE_DIR)
  include_directories(${SPDLOG_INCLUDE_DIR})
  add_executable(train ${TRAIN_DIR}/train.cpp ${COMMON_SOURCES})
  target_link_libraries(train PRIVATE ${COMMON_LIBS})
else()
  message(WARNING "spdlog headers not found; skipping train target.")
endif()

if(EXISTS "${TRAIN_DIR}/test.cpp")
  add_executable(test ${TRAIN_DIR}/test.cpp ${COMMON_SOURCES})
  target_link_libraries(test PRIVATE ${COMMON_LIBS})
endif()

add_executable(dsc ${TRAIN_DIR}/dsc.cpp ${COMMON_SOURCES})
target_link_libraries(dsc PRIVATE ${COMMON_LIBS})

add_executable(dsg ${TRAIN_DIR}/dsg.cpp ${TRAIN_DIR}/dsc.cpp ${TRAIN_DIR}/sandbox_mpc_torch.cpp ${COMMON_SOURCES})
target_link_libraries(dsg PRIVATE ${COMMON_LIBS})
target_compile_definitions(dsg PRIVATE DSG_BUILD)
