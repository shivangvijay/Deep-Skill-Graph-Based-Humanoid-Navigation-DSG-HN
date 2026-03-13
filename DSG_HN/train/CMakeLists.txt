cmake_minimum_required(VERSION 3.12)
project(dsg_train)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
#find_package(glfw REQUIRED)

set(MUJOCO_DIR "${PROJECT_SOURCE_DIR}/../../unitree_mujoco/simulate/mujoco")
set(POLICY_CONFIG_DIR "${PROJECT_SOURCE_DIR}/../deploy/robots/g1_29dof/config")
set(SCENE_XML_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1")

#set(SCENE_XML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1/ai_maker_space_scene.xml")
#set(G1_XML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_mujoco/unitree_robots/g1/g1_29dof.xml")

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  #include
  utils
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../deploy/include
  ${POLICY_CONFIG_DIR}
  ${MUJOCO_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../unitree_sdk2/include

  #${PROJECT_SOURCE_DIR}/../../include/
)

file(COPY ${SCENE_XML_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file (COPY ${G1_XML_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${POLICY_CONFIG_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/config)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw

)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_executable(train 
  main.cpp
  utils/mujoco_engine.cpp
)