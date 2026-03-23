add_library(robot
	src/ArmPosInitializer.cpp
	src/LegPosInitializer.cpp
	src/RobotController.cpp
	src/RobotModel.cpp
	src/RobotRunner.cpp
	../common/src/Controllers/ArmController.cpp
	../common/src/Controllers/LegController.cpp
)

target_include_directories(robot
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include/robot
		${PROJECT_SOURCE_DIR}/common/include/Utilities
)

target_link_libraries(robot PUBLIC
	common
)
