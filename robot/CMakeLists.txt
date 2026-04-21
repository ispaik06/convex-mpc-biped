add_library(robot
	src/ArmPosInitializer.cpp
	src/InitialPoseConfig.cpp
	src/JointTrackingConfig.cpp
	src/LegPosInitializer.cpp
	src/RobotRunner.cpp
)

find_package(yaml-cpp CONFIG REQUIRED)

target_include_directories(robot
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include/robot
)

target_link_libraries(robot PUBLIC
	common
	yaml-cpp::yaml-cpp
)

target_compile_definitions(robot
	PRIVATE
		PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
