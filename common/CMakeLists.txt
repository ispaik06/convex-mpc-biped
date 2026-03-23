add_library(common
	src/RobotModel.cpp
	src/Estimator/StateEstimator.cpp
)

target_include_directories(common PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/robot/include/robot
)

target_link_libraries(common PUBLIC
	Eigen3::Eigen
)
