add_library(robot
	src/ArmPosInitializer.cpp
	src/LegPosInitializer.cpp
	src/RobotRunner.cpp
)

target_include_directories(robot
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include/robot
)

target_link_libraries(robot PUBLIC
	common
)
