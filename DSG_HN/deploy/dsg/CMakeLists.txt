cmake_minimum_required(VERSION 3.12)
project(dsg)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

include_directories(
  #/usr/include/eigen3
  /opt/unitree_robotics/include
  /opt/unitree_robotics/include/ddscxx # dds
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  #${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../include/
)

set(SCENE_XML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_mujoco/unitree_robots/g1/ai_maker_space_scene.xml")
file(COPY ${SCENE_XML_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}) # file is for runtime, whereas include_directories is for compile time, where to find headers

link_directories(/opt/unitree_robotics/lib)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

#add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
#link_libraries(${PROJECT_NAME}_lib)

add_executable(example
  example.cpp
  robot_bridge.cpp
  ${PROJECT_SOURCE_DIR}/../include/dds/vel_publisher.cpp
  ${PROJECT_SOURCE_DIR}/../include/dds/vel_subscriber.cpp
  ${PROJECT_SOURCE_DIR}/../include/dds/state_subscriber.cpp
  ${PROJECT_SOURCE_DIR}/../include/dds/reset_publisher.cpp
)