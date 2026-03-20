add_library(robot
	src/ArmPosInitializer.cpp
	src/LegPosInitializer.cpp
	src/RobotController.cpp
	src/RobotModel.cpp
	src/RobotRunner.cpp
	src/main_helper.cpp
	../common/src/Controllers/ArmController.cpp
	../common/src/Controllers/LegController.cpp
)

target_include_directories(robot
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include/robot
		${PROJECT_SOURCE_DIR}/common/include/Utilities
	PRIVATE
		${PROJECT_SOURCE_DIR}/sim/include/sim
)

target_link_libraries(robot PUBLIC
	common
	sim
)
