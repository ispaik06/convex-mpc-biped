add_library(common
	src/Controllers/ArmController.cpp
	src/Controllers/LegController.cpp
	src/Dynamics/OperationalSpaceDynamics.cpp
	src/Robot/RobotModel.cpp
	src/Estimator/StateEstimator.cpp
	src/Utilities/KeyboardCommand.cpp
)

target_include_directories(common PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(common PUBLIC
	Eigen3::Eigen
)
