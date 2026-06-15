cmake_minimum_required(VERSION 3.16)
project(panda_joint_position_command LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(panda_joint_position
  src/main.cpp
)

target_link_libraries(panda_joint_position
  PRIVATE
    Franka::Franka
    Eigen3::Eigen
)

target_compile_options(panda_joint_position PRIVATE
  -Wall -Wextra -Wpedantic
)