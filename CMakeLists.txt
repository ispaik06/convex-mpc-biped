cmake_minimum_required(VERSION 3.16)
project(ConvexMPC LANGUAGES C CXX)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

# ---------- MuJoCo ----------
set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco" CACHE PATH "MuJoCo install prefix")
set(mujoco_DIR "${MUJOCO_ROOT}/lib/cmake/mujoco" CACHE PATH "Path to mujocoConfig.cmake")
find_package(mujoco CONFIG REQUIRED)

# ---------- Targets ----------
if(EXISTS "${CMAKE_SOURCE_DIR}/src/test.cpp")
  add_executable(test src/test.cpp)
  target_link_libraries(test PRIVATE Eigen3::Eigen mujoco::mujoco)
  set_target_properties(test PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
    INSTALL_RPATH "${MUJOCO_ROOT}/lib"
  )
endif()
