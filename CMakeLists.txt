cmake_minimum_required(VERSION 3.16)
project(ConvexMPC LANGUAGES C CXX)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

set(mujoco_DIR "$ENV{HOME}/.local/mujoco/lib/cmake/mujoco")
find_package(mujoco CONFIG REQUIRED)


add_subdirectory(common)
add_subdirectory(robot)
add_subdirectory(sim)
add_subdirectory(My_controller)
add_subdirectory(apps)
