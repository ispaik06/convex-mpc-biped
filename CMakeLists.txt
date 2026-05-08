cmake_minimum_required(VERSION 3.16)
project(convex_mpc_biped LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_subdirectory(common)
add_subdirectory(robot)
add_subdirectory(sim)
add_subdirectory(My_Controller)
add_subdirectory(apps)
add_subdirectory(test/gait_swing_hold)
add_subdirectory(test/gait_swing_hold_keyboard)
add_subdirectory(test/standing_debug)
add_subdirectory(test/scene_posture)
