add_library(sim
	src/MujocoCheaterStateReader.cpp
	src/main_thread.cpp
	src/SimViewer.cpp
	src/SimulationRunner.cpp
	src/setupRobotParams.cpp
	../robot/src/models/MitHumanoidSpec.cpp
	../robot/src/models/G1_23Spec.cpp
	../robot/src/models/H1Spec.cpp
)

target_include_directories(sim PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/sim
	${PROJECT_SOURCE_DIR}/robot/include/robot
)

target_include_directories(sim PRIVATE
	${PROJECT_SOURCE_DIR}/robot/include
)

target_link_libraries(sim
	PUBLIC
		common
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
