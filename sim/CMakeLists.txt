add_library(sim
	src/SimViewer.cpp
	src/SimulationRunner.cpp
)

target_include_directories(sim PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/sim
	${PROJECT_SOURCE_DIR}/robot/include/robot
)

target_link_libraries(sim
	PUBLIC
		common
		mujoco::mujoco
		glfw
)

target_compile_definitions(sim
	PRIVATE
		PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
