cmake_minimum_required(VERSION 3.12)
project(dsg_sandbox)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

set(SANDBOX_DIR "${PROJECT_SOURCE_DIR}/..")
set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.3.6")
set(POLICY_CONFIG_DIR "${SANDBOX_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/unitree_robots/g1")
set(JOYSTICK_SRC "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/simulate/src")
set(SIMULATE_SRC "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/simulate/mujoco/simulate")
set(SIMULATE_INCLUDE "${PROJECT_SOURCE_DIR}/../../../unitree_mujoco/simulate/mujoco")

include_directories(
  /opt/homebrew/include
  $ENV{CONDA_PREFIX}/include
  $ENV{CONDA_PREFIX}/include/eigen3
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include/ddscxx
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/thirdparty/include
  ${SANDBOX_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/include
  ${SANDBOX_DIR}/../deploy/include
  ${POLICY_CONFIG_DIR}
  ${MUJOCO_DIR}
  ${PROJECT_SOURCE_DIR}/../../../unitree_sdk2/include
  ${SANDBOX_DIR}/../robot_bridge/include
  ${JOYSTICK_SRC}
  ${JOYSTICK_SRC}/lodepng
  ${SIMULATE_INCLUDE}/include
  ${SIMULATE_SRC}
)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

set(SANDBOX_LIBS
  pthread
  /opt/homebrew/lib/libboost_program_options.dylib
  $ENV{CONDA_PREFIX}/lib/libyaml-cpp.dylib
  $ENV{CONDA_PREFIX}/lib/libfmt.dylib
  ${SANDBOX_DIR}/../deploy/thirdparty/onnxruntime-osx-arm64-1.22.0/lib/libonnxruntime.dylib
  ${MUJOCO_DIR}/lib/libmujoco.3.3.6.dylib
  $ENV{CONDA_PREFIX}/lib/libglfw.dylib
  "-framework CoreVideo"
  "-framework Cocoa"
  "-framework OpenGL"
)

file(GLOB SIMULATE_SOURCES
  ${SIMULATE_SRC}/glfw_*.cc
  ${SIMULATE_SRC}/platform_*.cc
  ${SIMULATE_SRC}/simulate.cc
)
# macOS-specific Objective-C++ files (not matched by *.cc glob)
list(APPEND SIMULATE_SOURCES
  ${SIMULATE_SRC}/glfw_corevideo.mm
  ${SIMULATE_SRC}/macos_gui.mm
)

add_executable(sandbox
  ${SANDBOX_DIR}/sandbox.cpp
  ${JOYSTICK_SRC}/joystick/joystick.cc
  ${JOYSTICK_SRC}/lodepng/lodepng.cpp
  ${SANDBOX_DIR}/../robot_bridge/mujoco_utils/mujoco_engine.cpp
  ${SANDBOX_DIR}/../robot_bridge/robot_bridge.cpp
  ${SANDBOX_DIR}/../robot_bridge/robot_bridge_train.cpp
  ${SIMULATE_SOURCES}
)
target_link_libraries(sandbox PRIVATE ${SANDBOX_LIBS})
