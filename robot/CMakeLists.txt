add_library(robot
	src/RobotModel.cpp
	src/RobotRunner.cpp
	src/main_helper.cpp
	../common/src/Controllers/LegController.cpp
)

target_include_directories(robot
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include/robot
	PRIVATE
		${PROJECT_SOURCE_DIR}/sim/include/sim
)

target_link_libraries(robot PUBLIC
	common
	sim
)
