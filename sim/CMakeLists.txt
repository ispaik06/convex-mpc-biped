add_library(sim
	src/LegSwingDynamicsProvider.cpp
	src/MujocoCheaterStateReader.cpp
	src/SimulationConfig.cpp
	src/models/MitHumanoidSpec.cpp
	src/models/RobotMujocoSpec.cpp
	src/models/UnitreeG1Spec.cpp
	src/models/UnitreeH1Spec.cpp
	src/main_thread.cpp
	src/SimulationRunner.cpp
	src/setupRobotParams.cpp
)

target_include_directories(sim PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/sim
	${PROJECT_SOURCE_DIR}/robot/include/robot
)

target_include_directories(sim PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sim
	PUBLIC
		common
		robot
		mujoco::libmujoco_simulate
		mujoco::mujoco
		glfw
)

if(APPLE)
	target_link_libraries(sim PUBLIC "-framework CoreVideo")
endif()

target_compile_definitions(sim
	PRIVATE
		PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
